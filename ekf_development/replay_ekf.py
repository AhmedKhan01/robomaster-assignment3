#!/usr/bin/env python3
"""Replay five pilot logs through an EKF; never import ROS or connect to a robot."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics as st
from ekf_core import EKF,relative_pose


def unpack(row):
    d=json.loads(row['payload_json'])
    if isinstance(d,dict) and isinstance(d.get('data'),str):
        try:return json.loads(d['data'])
        except json.JSONDecodeError:
            try:return json.loads(d['data'].replace('\\"','"'))
            except json.JSONDecodeError:pass
    return d


def replay(folder,measurement,cfg):
    metadata=json.loads((folder/'metadata_original.json').read_text())
    with (folder/'raw_events.csv').open() as f:rows=list(csv.DictReader(f))
    start=metadata['motion_start_elapsed_s'];stop=metadata['stop_elapsed_s']
    if metadata['result']!='COMPLETED_TIMED_SEGMENT' or start is None:
        raise ValueError('Replay requires a completed calibration pilot.')
    events=[(float(r['elapsed_s']),r['topic_or_event'],unpack(r),float(r['receipt_ros_s'])) for r in rows]
    init=next(d for _,topic,d,_ in events if topic=='MOTION_START')
    origin=init['initial_odom'];q=origin['quaternion'];yaw=2*math.atan2(q[2],q[3])
    r0=init['initial_tof']['range']
    ranges=[d['data'][0] for t,topic,d,_ in events if topic=='/rm/range' and start-.8<=t<start]
    r0_variance=max(.002**2,st.variance(ranges) if len(ranges)>1 else 0.)
    ekf=EKF(cfg);control=(0.,0.);raw_x=0.;tof_x=0.;history=[];last_record=-math.inf
    for elapsed,topic,d,ros in events:
        if elapsed<start:continue
        t=elapsed-start
        ekf.predict_to(t,control)
        if topic=='/rm/applied_cmd':
            control=(d['linear']['x'],d['angular']['z'])
        elif topic=='/rm/odom':
            p=d['pose']['pose'];q=p['orientation'];s=d['header']['stamp']
            age=ros-(s['sec']+s['nanosec']*1e-9)
            pos=relative_pose([p['position']['x'],p['position']['y']],
                              [q[k] for k in ('x','y','z','w')],origin['position'],yaw)
            raw_x=float(pos[0])
            ekf.observe_odom(pos[0],pos[1],t-age,age)
        elif topic=='/rm/range':
            distance,age=d['data'];tof_x=cfg['tof_travel_scale']*(r0-distance)
            ekf.observe_tof(tof_x,t-age,age,r0_variance)
        if elapsed-last_record>=.045:
            history.append(dict(time_from_motion_s=t,raw_odom_m=raw_x,scaled_tof_m=tof_x,
                ekf_x_m=float(ekf.state[0]),ekf_y_m=float(ekf.state[1]),
                ekf_theta_rad=float(ekf.state[2]),ekf_v_m_s=float(ekf.state[3]),
                ekf_omega_rad_s=float(ekf.state[4]),P_xx_m2=float(ekf.P[0,0]),
                applied_v_m_s=control[0],applied_omega_rad_s=control[1],
                settled=elapsed>=stop+1.))
            last_record=elapsed
    settled=[r for r in history if r['settled']][-10:]
    if len(settled)<5:raise ValueError('Insufficient settled replay samples.')
    physical=measurement['start_gap_m']-measurement['final_gap_m']
    estimate=st.median(r['ekf_x_m'] for r in settled)
    return history,dict(run_id=metadata['run_id'],stage='in_sample_calibration_replay',
        physical_travel_m=physical,ekf_settled_m=estimate,ekf_error_m=estimate-physical,
        accepted_odom=ekf.counts['odom_accepted'],accepted_tof=ekf.counts['tof_accepted'],
        skipped_measurements=ekf.counts['rejected'],
        r0_sample_spread_variance_m2=r0_variance,
        note='No evaluation evidence. Q/R assumptions remain provisional; delayed updates are applied at receipt, without rewind.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration-dir',required=True,type=Path)
    parser.add_argument('--config',type=Path,default=Path(__file__).with_name('ekf_config.json'))
    parser.add_argument('--output',type=Path,default=Path('replay01'))
    args=parser.parse_args();cfg=json.loads(args.config.read_text())
    physical=json.loads((args.calibration_dir/'physical_measurements.json').read_text())
    args.output.mkdir(parents=True,exist_ok=False)
    summary=[]
    for measurement in physical['runs']:
        run=measurement['run_id'];history,result=replay(args.calibration_dir/'runs'/run,measurement,cfg)
        with (args.output/(run+'_states.csv')).open('w',newline='') as f:
            writer=csv.DictWriter(f,list(history[0]));writer.writeheader();writer.writerows(history)
        summary.append(result)
        print(f"{run}: physical={result['physical_travel_m']:.3f} m; EKF={result['ekf_settled_m']:.3f} m; error={100*result['ekf_error_m']:+.2f} cm")
    (args.output/'replay_summary.json').write_text(json.dumps(dict(evaluation_ready=False,runs=summary),indent=2)+'\n')
    print('Calibration replay only. No robot commands were sent.')


if __name__=='__main__':main()
