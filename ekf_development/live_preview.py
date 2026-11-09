#!/usr/bin/env python3
"""Stationary EKF preview. Publishes diagnostic String only; NO command publisher."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import time
from ekf_core import EKF,relative_pose


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id',required=True)
    parser.add_argument('--duration',type=float,default=20.)
    parser.add_argument('--config',type=Path,default=Path(__file__).with_name('ekf_config.json'))
    parser.add_argument('--output-root',type=Path,default=Path('live_previews'))
    args=parser.parse_args()
    if Path(args.run_id).name!=args.run_id or args.run_id in ('.','..'):
        parser.error('Use a simple run ID.')
    if not math.isfinite(args.duration) or not 5<=args.duration<=60:
        parser.error('Require 5 <= duration <= 60 s.')
    cfg=json.loads(args.config.read_text())
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String,Float64MultiArray
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    folder=args.output_root/args.run_id;folder.mkdir(parents=True,exist_ok=False)
    stream=(folder/'events.csv').open('x',newline='')
    writer=csv.writer(stream);writer.writerow(['elapsed_s','event','payload_json'])

    class Preview(Node):
        def __init__(self):
            super().__init__('assessment3_ekf_preview')
            self.started=time.monotonic();self.inputs={};self.ekf=None
            self.origin=None;self.r0=None;self.healthy_since=None
            self.range_history=[];self.r0_variance=.002**2
            self.epoch=None;self.done=False;self.clock_fault=False
            self.last_clock=None;self.last_print=-math.inf;self.status='STARTING'
            self.pub=self.create_publisher(String,'/rm/ekf_preview',10)
            self.subs=[]
            for topic,typ in [('/rm/odom',Odometry),('/rm/range',Float64MultiArray),
                              ('/rm/tag_observation',String),('/rm/driver_status',String),
                              ('/rm/applied_cmd',Twist)]:
                self.subs.append(self.create_subscription(typ,topic,
                    lambda msg,name=topic:self.receive(name,msg),qos_profile_sensor_data))
            self.timer=self.create_timer(.05,self.tick)
            self.get_logger().info('STATIONARY PREVIEW: no command publisher. Output: '+str(folder.resolve()))

        def log(self,event,payload):
            writer.writerow([time.monotonic()-self.started,event,json.dumps(payload,allow_nan=False)])

        def receive(self,topic,msg):
            mono=time.monotonic();ros=self.get_clock().now().nanoseconds*1e-9
            key={'/rm/odom':'odom','/rm/range':'tof','/rm/tag_observation':'tag',
                 '/rm/driver_status':'driver','/rm/applied_cmd':'applied'}[topic]
            try:
                if key=='odom':
                    p=msg.pose.pose.position;q=msg.pose.pose.orientation
                    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                    item=dict(position=[p.x,p.y],quaternion=[q.x,q.y,q.z,q.w],age=ros-stamp)
                    if not all(math.isfinite(x) for x in item['position']+item['quaternion']+[item['age']]):
                        raise ValueError('Nonfinite pose')
                    if abs(sum(x*x for x in item['quaternion'])-1)>.1:
                        raise ValueError('Invalid quaternion')
                elif key=='tof':
                    if len(msg.data)!=2:raise ValueError('Invalid range layout')
                    item=dict(range=float(msg.data[0]),age=float(msg.data[1]))
                    if not all(math.isfinite(x) for x in item.values()):raise ValueError('Nonfinite range')
                elif key=='tag':
                    d=json.loads(msg.data);o=d.get('observation') or {}
                    item=dict(visible=d.get('visible') is True,target_id=d.get('target_id'),
                              error=o.get('horizontal_error_normalized'),
                              age=None if not o else ros-o['source_stamp_ros_s'])
                elif key=='driver':item=dict(status=msg.data)
                else:
                    item=dict(v=msg.linear.x,w=msg.angular.z)
                    if not math.isfinite(item['v']+item['w']):raise ValueError('Nonfinite applied command')
                item['receipt']=mono;self.inputs[key]=item
                if key=='tof':
                    self.range_history.append((mono,item['range']))
                    self.range_history=[p for p in self.range_history if mono-p[0]<=1.]
                self.log(topic,item)
                if self.ekf is not None and not self.clock_fault:
                    t=mono-self.epoch
                    self.ekf.predict_to(t,(0.,0.))
                    if key=='odom':
                        p=relative_pose(item['position'],item['quaternion'],self.origin[0],self.origin[1])
                        self.ekf.observe_odom(p[0],p[1],t-item['age'],item['age'])
                    elif key=='tof' and .10<=item['range']<=10:
                        self.ekf.observe_tof(cfg['tof_travel_scale']*(self.r0-item['range']),t-item['age'],item['age'],self.r0_variance)
            except (ValueError,KeyError,TypeError,OverflowError) as exc:
                self.inputs.pop(key,None);self.log('INPUT_REJECTED',dict(topic=topic,error=str(exc)))

        def health(self,now):
            if self.clock_fault:return 'CLOCK_JUMP_RESTART_PREVIEW'
            if self.count_publishers('/rm/cmd_vel')!=0:return 'EXISTING_COMMAND_PUBLISHER'
            for key,limit in [('tof',.4),('odom',.4),('tag',.3),('driver',.4),('applied',.4)]:
                item=self.inputs.get(key)
                if item is None:return key.upper()+'_MISSING'
                receipt_age=now-item['receipt']
                if not 0<=receipt_age<=limit:return key.upper()+'_RECEIPT_STALE'
                if key in ('tof','odom') and not -.05<=item['age']+receipt_age<=limit:
                    return key.upper()+'_SOURCE_STALE'
            if not .30<self.inputs['tof']['range']<=10:return 'TOF_RANGE_GUARD'
            tag=self.inputs['tag']
            if not tag['visible'] or tag['target_id']!=3:return 'TAG3_NOT_VISIBLE'
            if tag['age'] is None or not -.05<=tag['age']+now-tag['receipt']<=.3:return 'TAG_SOURCE_STALE'
            if tag['error'] is None or not math.isfinite(tag['error']) or abs(tag['error'])>.15:return 'TAG_NOT_CENTERED'
            if self.inputs['driver']['status'] not in ('READY','COMMAND_TIMEOUT'):
                return 'DRIVER_'+self.inputs['driver']['status']
            if abs(self.inputs['applied']['v'])>1e-6 or abs(self.inputs['applied']['w'])>1e-6:
                return 'NONZERO_APPLIED_COMMAND'
            return None

        def tick(self):
            now=time.monotonic();ros=self.get_clock().now().nanoseconds*1e-9
            if self.last_clock:
                pm,pr=self.last_clock
                step=(ros-pr)-(now-pm)
                if abs(step)>.05:
                    self.clock_fault=True;self.log('CLOCK_JUMP',dict(step_s=step))
            self.last_clock=(now,ros)
            issue=self.health(now)
            if issue:self.healthy_since=None
            elif self.healthy_since is None:self.healthy_since=now
            if self.ekf is None and self.healthy_since is not None and now-self.healthy_since>=1:
                o=self.inputs['odom'];q=o['quaternion']
                yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]))
                self.origin=(o['position'],yaw);self.r0=self.inputs['tof']['range']
                ranges=[p[1] for p in self.range_history]
                self.r0_variance=max(.002**2,statistics.variance(ranges) if len(ranges)>1 else 0.)
                self.epoch=now;self.ekf=EKF(cfg)
                self.log('ORIGIN',dict(position=o['position'],yaw_rad=yaw,r0_m=self.r0,r0_variance_m2=self.r0_variance))
            self.status=issue or ('HEALTHY' if self.ekf else 'INITIALIZING_ORIGIN')
            details=dict(status=self.status,commands_published=0,evaluation_ready=False)
            if self.ekf:
                if not self.clock_fault:self.ekf.predict_to(now-self.epoch,(0.,0.))
                details.update(state=self.ekf.state.tolist(),covariance_diagonal=self.ekf.P.diagonal().tolist(),
                               accepted_counts=self.ekf.counts.copy())
            self.pub.publish(String(data=json.dumps(details,allow_nan=False)))
            self.log('PREVIEW',details)
            if now-self.last_print>=1:
                x='' if not self.ekf else f'; x={self.ekf.state[0]:+.4f} m'
                self.get_logger().info(self.status+x);self.last_print=now;stream.flush()
            if now-self.started>=args.duration:self.done=True

    node=None;rclpy.init()
    try:
        node=Preview()
        while rclpy.ok() and not node.done:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:pass
    finally:
        summary=dict(stage='stationary_ekf_preview',commands_published=0,evaluation_ready=False)
        if node:
            summary.update(last_status=node.status,clock_jump_detected=node.clock_fault,
                           initialized=node.ekf is not None,
                           state=None if node.ekf is None else node.ekf.state.tolist())
            node.destroy_node()
        stream.close();(folder/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        if rclpy.ok():rclpy.shutdown()
        print('Saved:',folder.resolve())


if __name__=='__main__':main()
