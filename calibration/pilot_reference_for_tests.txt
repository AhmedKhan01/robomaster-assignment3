#!/usr/bin/env python3
"""Bounded ROS 2 speed-characterization pilot. Preview is the default.

This is NOT the final reference-tracking controller and NOT an evaluation run.
"""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from pilot_core import check_health, command_at


def clean(x):
    if isinstance(x,float) and not math.isfinite(x): return None
    if isinstance(x,dict): return {k:clean(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)): return [clean(v) for v in x]
    return x


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-id',required=True)
    p.add_argument('--start-gap',type=float,required=True,help='Measured robot front to box face in metres')
    p.add_argument('--speed',type=float,default=.04)
    p.add_argument('--duration',type=float,default=10.)
    p.add_argument('--execute',action='store_true',help='Enable this timed motion segment after a five-second healthy countdown')
    p.add_argument('--output-root',default='pilot_runs')
    args=p.parse_args()
    if not all(math.isfinite(v) for v in (args.start_gap,args.speed,args.duration)):
        p.error('Numeric inputs must be finite')
    if not 0 < args.speed <= .12 or not 0 < args.duration <= 30:
        p.error('Require 0 < speed <= .12 m/s and 0 < duration <= 30 seconds')
    if args.start_gap < .6 or args.speed*args.duration > args.start_gap-.50:
        p.error('Nominal timed travel must leave at least 0.50 m physical clearance')
    if Path(args.run_id).name != args.run_id or args.run_id in ('.','..'):
        p.error('Use a simple run-id directory name')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float64MultiArray,String
    from rosidl_runtime_py.convert import message_to_ordereddict

    folder=Path(args.output_root).expanduser()/args.run_id
    folder.mkdir(parents=True,exist_ok=False)
    metadata=dict(vars(args),stage='speed_characterization_pilot',evaluation_run=False,
        created_utc=datetime.now(timezone.utc).isoformat(),
        commanded_nominal_travel_m=args.speed*args.duration,
        physical_start_gap_m=args.start_gap,physical_final_gap_m=None,
        physical_measurement_resolution_m=None,
        camera_geometry='upright tag; normalized horizontal image error; no camera distance',
        guard_raw_tof_m=.30,driver_guard_note='Existing driver ToF latch and timeout remain independent',
        result='RUNNING',stop_reason=None)
    metafile=folder/'metadata.json'
    metafile.write_text(json.dumps(metadata,indent=2))
    stream=(folder/'raw_events.csv').open('x',newline='')
    writer=csv.writer(stream)
    writer.writerow(['elapsed_s','receipt_ros_s','topic_or_event','payload_json'])

    class Pilot(Node):
        def __init__(self):
            super().__init__('assessment3_calibration_pilot')
            self.start=time.monotonic()
            self.inputs={}
            self.phase='PREVIEW' if not args.execute else 'WAITING_FOR_SENSORS'
            self.healthy_since=None
            self.motion_start=None
            self.stop_time=None
            self.reason=None
            self.done=False
            self.last_print=-math.inf
            self.cmd_pub=self.create_publisher(Twist,'/rm/cmd_vel',1) if args.execute else None
            self.subs=[]
            for topic,typ in [('/rm/range',Float64MultiArray),('/rm/odom',Odometry),
                    ('/rm/tag_observation',String),('/rm/driver_status',String),
                    ('/rm/applied_cmd',Twist),('/rm/odom_raw',String),('/rm/wheels',Float64MultiArray)]:
                self.subs.append(self.create_subscription(typ,topic,
                    lambda msg,name=topic:self.receive(name,msg),qos_profile_sensor_data))
            self.create_timer(.05,self.tick)
            self.get_logger().info(('MOTION ENABLED: five-second healthy countdown.' if args.execute
                else 'PREVIEW ONLY: no command publisher created.')+' Output: '+str(folder.resolve()))

        def log(self,topic,payload):
            writer.writerow([time.monotonic()-self.start,self.get_clock().now().nanoseconds*1e-9,
                             topic,json.dumps(clean(payload),allow_nan=False,separators=(',',':'))])

        def receive(self,topic,msg):
            now=time.monotonic()
            self.log(topic,message_to_ordereddict(msg))
            try:
                if topic=='/rm/range':
                    if len(msg.data)!=2: raise ValueError('Invalid range layout')
                    self.inputs['tof']=dict(receipt=now,range=float(msg.data[0]),sensor_age=float(msg.data[1]))
                elif topic=='/rm/odom':
                    pos=msg.pose.pose.position;q=msg.pose.pose.orientation
                    self.inputs['odom']=dict(receipt=now,position=(pos.x,pos.y),
                        quaternion=(q.x,q.y,q.z,q.w),
                        source_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
                elif topic=='/rm/tag_observation':
                    d=json.loads(msg.data); obs=d.get('observation') or {}
                    self.inputs['tag']=dict(receipt=now,visible=d.get('visible') is True,
                        target_id=d.get('target_id'),error=obs.get('horizontal_error_normalized'),
                        source_stamp=obs.get('source_stamp_ros_s',math.nan))
                elif topic=='/rm/driver_status':
                    self.inputs['driver']=dict(receipt=now,status=msg.data)
            except (ValueError,TypeError,KeyError) as exc:
                key={'/rm/range':'tof','/rm/odom':'odom','/rm/tag_observation':'tag','/rm/driver_status':'driver'}.get(topic)
                if key:self.inputs.pop(key,None)
                self.log('MALFORMED_INPUT',dict(topic=topic,error=str(exc)))

        def publish_command(self,v=0.,w=0.):
            if self.cmd_pub is not None:
                msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w)
                self.cmd_pub.publish(msg)
                self.log('REQUESTED_COMMAND',dict(v_m_s=v,omega_rad_s=w,phase=self.phase))

        def stop(self,reason):
            if self.stop_time is None:
                self.reason=reason;self.stop_time=time.monotonic()
                self.phase='SETTLING'
                self.log('STOP',dict(reason=reason,motion_start=self.motion_start))
            self.publish_command()

        def tick(self):
            now=time.monotonic();ros_now=self.get_clock().now().nanoseconds*1e-9
            issue=check_health(now,ros_now,self.inputs,self.count_publishers('/rm/cmd_vel'),args.execute)
            if self.stop_time is not None:
                self.publish_command()
                if now-self.stop_time >= 1.5:self.done=True
                return
            if now-self.start > args.duration+20:
                self.stop('BOUNDED_RUNTIME');return
            if not args.execute:
                if now-self.last_print>=1:
                    self.get_logger().info('PREVIEW: '+('HEALTHY' if issue is None else issue))
                    self.last_print=now;stream.flush()
                if now-self.start>=8:
                    self.reason='PREVIEW_HEALTHY' if issue is None else 'PREVIEW_'+issue
                    self.done=True
                return
            if self.motion_start is None:
                self.publish_command()
                if issue:
                    self.healthy_since=None
                    if now-self.last_print>=1:
                        self.get_logger().info('WAITING: '+issue)
                        self.last_print=now
                    if now-self.start>=12:self.stop('PRECHECK_'+issue)
                    return
                if self.healthy_since is None:self.healthy_since=now
                self.phase='COUNTDOWN'
                if now-self.last_print>=1:
                    self.get_logger().info('Healthy countdown: '+str(round(max(0,5-(now-self.healthy_since)),1))+' s')
                    self.last_print=now
                if now-self.healthy_since<5:return
                self.motion_start=now;self.phase='MOVING'
                self.log('MOTION_START',dict(physical_start_gap_m=args.start_gap,
                    initial_odom=self.inputs['odom'],initial_tof=self.inputs['tof']))
            if issue:
                self.stop(issue);return
            elapsed=now-self.motion_start
            if elapsed>=args.duration:
                self.stop('TIMED_SEGMENT_COMPLETE');return
            error=self.inputs['tag']['error']
            v,w=command_at(elapsed,args.duration,args.speed,error)
            self.publish_command(v,w)
            if now-self.last_print>=1:
                self.get_logger().info(f'MOVING: elapsed={elapsed:.1f}s, v={v:.3f}m/s, omega={w:.3f}rad/s')
                self.last_print=now;stream.flush()

    node=None;rclpy.init()
    try:
        node=Pilot()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:
        if node:node.stop('OPERATOR_CTRL_C')
    except BaseException as exc:
        if node:node.stop('EXCEPTION_'+type(exc).__name__)
        metadata['exception']=repr(exc)
        raise
    finally:
        if node:
            if args.execute:
                # Several stop messages; driver also has its independent command timeout.
                try:
                    deadline=time.monotonic()+.3
                    while rclpy.ok() and time.monotonic()<deadline:
                        node.publish_command();rclpy.spin_once(node,timeout_sec=.05)
                except Exception:pass
            metadata.update(stop_reason=node.reason or 'PROCESS_ENDED',
                result='PREVIEW' if not args.execute else
                    ('COMPLETED_TIMED_SEGMENT' if node.reason=='TIMED_SEGMENT_COMPLETE' else 'ABORTED'),
                motion_start_elapsed_s=None if node.motion_start is None else node.motion_start-node.start,
                stop_elapsed_s=None if node.stop_time is None else node.stop_time-node.start,
                duration_s=time.monotonic()-node.start,last_inputs=clean(node.inputs))
            node.destroy_node()
        stream.flush();stream.close()
        metadata['closed_utc']=datetime.now(timezone.utc).isoformat()
        metafile.write_text(json.dumps(clean(metadata),indent=2,allow_nan=False))
        if rclpy.ok():rclpy.shutdown()
        print('Saved:',folder.resolve())
        if args.execute:print('Measure settled final physical front gap. Stop driver before manually repositioning the robot.')

if __name__=='__main__':main()
