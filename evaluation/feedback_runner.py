#!/usr/bin/env python3
"""Shared unfiltered/EKF approach controller. Default: no-motion preview.

Evaluation requires a verified frozen manifest; preview remains the default.
"""
import argparse
import csv
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path
import time
from feedback_core import Session,reference


def clean(value):
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {k:clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean(v) for v in value]
    return value


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id',required=True)
    parser.add_argument('--method',choices=['unfiltered','filtered'],required=True)
    parser.add_argument('--start-gap',type=float,required=True,help='Measured physical front-edge-to-box-face gap, metres')
    parser.add_argument('--execute',action='store_true',help='Enable motion after five continuously healthy seconds')
    parser.add_argument('--evaluation',action='store_true',help='Require frozen protocol and label an executed run as evaluation')
    parser.add_argument('--output-root',type=Path,default=Path('feedback_runs'))
    args=parser.parse_args()
    if Path(args.run_id).name!=args.run_id or args.run_id in ('.','..'):parser.error('Use a simple run ID.')
    if not math.isfinite(args.start_gap) or not .60<=args.start_gap<=2.50:parser.error('Measured start gap must be 0.60 to 2.50 m.')
    root=Path(__file__).resolve().parent
    protocol=json.loads((root/'protocol.json').read_text())
    cfg=json.loads((root/'ekf_config.json').read_text())
    frozen_sha=None;slot=None;freeze=None
    if args.evaluation:
        from freeze_protocol import verify,planned_slot
        freeze,frozen_sha=verify(root)
        if not freeze['minimum_start_gap_m']<=args.start_gap<=freeze['maximum_start_gap_m']:
            parser.error('Reposition at the marked 0.60 m start and enter the actual measurement (0.60–0.62 m).')
        if args.execute:
            slot=planned_slot(args.run_id,args.method)
            previous=list(args.output_root.glob(slot['slot']+'*/metadata.json'))
            for candidate in previous:
                old=json.loads(candidate.read_text())
                if old.get('planned_slot')==slot['slot'] and old.get('result')=='COMPLETED_EVALUATION':
                    parser.error('This slot already completed. Record its physical endpoint; do not replace it with a better run.')
    source_hashes={n:hashlib.sha256((root/n).read_bytes()).hexdigest() for n in
                   ['feedback_core.py','feedback_runner.py','ekf_core.py','protocol.json','ekf_config.json']}
    from ekf_core import EKF
    EKF(cfg)  # validate before creating ROS entities
    Session(args.method,args.start_gap,args.execute,protocol,cfg,0.)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String,Float64MultiArray
    from rosidl_runtime_py.convert import message_to_ordereddict
    folder=args.output_root/args.run_id;folder.mkdir(parents=True,exist_ok=False)
    metadata=dict(run_id=args.run_id,method=args.method,execute=args.execute,
        stage='evaluation' if args.evaluation and args.execute else ('evaluation_preview' if args.evaluation else 'commissioning_only'),
        evaluation_run=bool(args.evaluation and args.execute),evaluation_ready=bool(args.evaluation),
        frozen_manifest_sha256=frozen_sha,planned_slot=None if slot is None else slot['slot'],
        attempt=None if slot is None else slot['attempt'],
        created_utc=datetime.now(timezone.utc).isoformat(),physical_start_gap_m=args.start_gap,
        physical_final_gap_m=None,physical_measurement_resolution_m=None,
        protocol=protocol,ekf_config=cfg,source_sha256=source_hashes,
        camera_geometry='upright tag; normalized pixel centering; no camera-derived distance',
        result='RUNNING',stop_reason=None)
    (folder/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    stream=(folder/'raw_events.csv').open('x',newline='');writer=csv.writer(stream)
    writer.writerow(['elapsed_s','receipt_ros_s','topic_or_event','payload_json'])

    class Runner(Node):
        def __init__(self):
            super().__init__('assessment3_feedback')
            self.started=time.monotonic();self.last_print=-math.inf;self.last_phase=None
            self.command_pub=self.create_publisher(Twist,'/rm/cmd_vel',1) if args.execute else None
            self.detail_pub=self.create_publisher(String,'/rm/feedback_status',10)
            self.session=Session(args.method,args.start_gap,args.execute,protocol,cfg,self.started,
                                 emit=self.log,on_stop=lambda:self.publish_command(0.,0.))
            self.subs=[]
            for topic,typ in [('/rm/odom',Odometry),('/rm/range',Float64MultiArray),
                ('/rm/tag_observation',String),('/rm/driver_status',String),
                ('/rm/applied_cmd',Twist),('/rm/odom_raw',String),('/rm/wheels',Float64MultiArray)]:
                self.subs.append(self.create_subscription(typ,topic,
                    lambda msg,name=topic:self.receive(name,msg),qos_profile_sensor_data))
            self.timer=self.create_timer(.05,self.tick)
            ref=reference(0.,args.start_gap)
            banner=('EVALUATION MOTION ENABLED' if args.evaluation else 'COMMISSIONING MOTION ENABLED') if args.execute else 'PREVIEW ONLY: no command publisher'
            self.get_logger().info(f'{banner}; method={args.method}; L={ref["L"]:.3f} m; T={ref["T"]:.2f} s; raw ToF guard={protocol["raw_tof_stop_m"]:.3f} m')
            self.get_logger().info('Output: '+str(folder.resolve()))

        def log(self,topic,payload):
            writer.writerow([time.monotonic()-self.started,self.get_clock().now().nanoseconds*1e-9,
                topic,json.dumps(clean(payload),allow_nan=False,separators=(',',':'))])

        def publish_command(self,v,w):
            if self.command_pub is None:return
            msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w)
            self.command_pub.publish(msg)
            self.log('REQUESTED_COMMAND',dict(v_m_s=v,omega_rad_s=w,phase=self.session.phase))

        def receive(self,topic,msg):
            self.log(topic,message_to_ordereddict(msg))
            now=time.monotonic();ros=self.get_clock().now().nanoseconds*1e-9
            key={'/rm/odom':'odom','/rm/range':'tof','/rm/tag_observation':'tag',
                 '/rm/driver_status':'driver','/rm/applied_cmd':'applied'}.get(topic)
            if key is None:return
            try:
                if key=='odom':
                    p=msg.pose.pose.position;q=msg.pose.pose.orientation
                    item=dict(position=[float(p.x),float(p.y)],quaternion=[float(q.x),float(q.y),float(q.z),float(q.w)],
                        source_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
                    if not all(math.isfinite(v) for v in item['position']+item['quaternion']+[item['source_stamp']]):raise ValueError('Nonfinite odometry')
                    if abs(sum(v*v for v in item['quaternion'])-1)>.1:raise ValueError('Invalid quaternion')
                elif key=='tof':
                    if len(msg.data)!=2:raise ValueError('Unexpected range layout')
                    item=dict(range=float(msg.data[0]),sensor_age=float(msg.data[1]))
                    if not math.isfinite(item['range']+item['sensor_age']):raise ValueError('Nonfinite ToF')
                elif key=='tag':
                    d=json.loads(msg.data);o=d.get('observation') or {}
                    item=dict(visible=d.get('visible') is True,target_id=d.get('target_id'),
                        error=o.get('horizontal_error_normalized'),source_stamp=o.get('source_stamp_ros_s',math.nan))
                    if item['error'] is not None:item['error']=float(item['error'])
                    item['source_stamp']=float(item['source_stamp'])
                elif key=='driver':item=dict(status=msg.data)
                else:
                    item=dict(v=float(msg.linear.x),w=float(msg.angular.z))
                    if not math.isfinite(item['v']+item['w']):raise ValueError('Nonfinite applied command')
                item['receipt']=now
                self.session.receive(key,item,now,ros,self.count_publishers('/rm/cmd_vel'))
            except (ValueError,KeyError,TypeError,OverflowError) as exc:
                self.session.invalidate(key,now)
                self.log('MALFORMED_INPUT',dict(topic=topic,error=str(exc)))

        def tick(self):
            now=time.monotonic();ros=self.get_clock().now().nanoseconds*1e-9
            data=self.session.tick(now,ros,self.count_publishers('/rm/cmd_vel'))
            if data['phase']!=self.last_phase:
                self.log('STATE_TRANSITION',dict(previous=self.last_phase,current=data['phase']))
                self.last_phase=data['phase']
            self.publish_command(data['requested_v_m_s'],data['requested_omega_rad_s'])
            self.log('FEEDBACK_STATE',data)
            self.detail_pub.publish(String(data=json.dumps(clean(data),allow_nan=False)))
            if now-self.last_print>=1:
                self.get_logger().info(f'{data["phase"]}: {data["status"]}; v={data["requested_v_m_s"]:.3f} m/s; raw={data["raw_odom_progress_m"]}; EKF={data.get("filtered_progress_m")}; countdown={data.get("countdown_remaining_s")}')
                self.last_print=now;stream.flush()

    node=None;rclpy.init()
    try:
        node=Runner()
        while rclpy.ok() and not node.session.done:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:
        if node:node.session.stop('OPERATOR_CTRL_C',time.monotonic())
    except BaseException as exc:
        metadata['exception']=repr(exc)
        if node:node.session.stop('EXCEPTION_'+type(exc).__name__,time.monotonic())
        raise
    finally:
        if node:
            if args.execute:
                # First final zero is immediate. Independent driver timeout remains.
                deadline=time.monotonic()+.3
                try:
                    while rclpy.ok() and time.monotonic()<deadline:
                        node.publish_command(0.,0.);rclpy.spin_once(node,timeout_sec=.05)
                except Exception:pass
            s=node.session;reason=s.stop_reason or 'PROCESS_ENDED'
            result='PREVIEW' if not args.execute else (('COMPLETED_EVALUATION' if args.evaluation else 'COMPLETED_COMMISSIONING') if s.motion_start is not None and reason in ('RAW_TOF_GUARD','REFERENCE_REACHED') else 'ABORTED')
            metadata.update(result=result,stop_reason=reason,
                motion_start_elapsed_s=None if s.motion_start is None else s.motion_start-s.started,
                stop_elapsed_s=None if s.stop_time is None else s.stop_time-s.started,
                clock_jump_detected=s.clock_fault,preview_issue_counts=s.preview_issues,
                final_state=s.details(time.monotonic(),'FINISHED',0.,0.))
            node.destroy_node()
        metadata['closed_utc']=datetime.now(timezone.utc).isoformat()
        stream.flush();stream.close()
        (folder/'metadata.json').write_text(json.dumps(clean(metadata),indent=2,allow_nan=False)+'\n')
        if rclpy.ok():rclpy.shutdown()
        print('Saved:',folder.resolve())
        if args.execute:print('Measure the settled physical final front gap before reset. Stop driver before manual repositioning.')


if __name__=='__main__':main()
