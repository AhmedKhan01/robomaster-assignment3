#!/usr/bin/env python3
"""Offline calibration from recorded pilots. No ROS, SDK or movement commands.

Physical measurements come from physical_measurements.json, not from ToF.
The original recorder metadata and CSVs remain unchanged.
"""
import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics as st


def unpack(row):
    value = json.loads(row['payload_json'])
    if isinstance(value, dict) and isinstance(value.get('data'), str):
        raw = value['data']
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            try:
                return json.loads(raw.replace('\\"', '"'))
            except json.JSONDecodeError:
                pass
    return value


def spread(values):
    center = st.median(values)
    return dict(n=len(values), mean=st.mean(values), median=center,
                sample_sd=st.stdev(values) if len(values) > 1 else None,
                robust_sd_1p4826_MAD=1.4826 * st.median(abs(x-center) for x in values),
                minimum=min(values), maximum=max(values))


def slope(points):
    if len(points) < 3:
        return None
    tx, xx = zip(*points)
    tm, xm = st.mean(tx), st.mean(xx)
    denominator = sum((t-tm)**2 for t in tx)
    return sum((t-tm)*(x-xm) for t,x in points)/denominator if denominator else None


def analyze_run(folder, physical):
    metadata = json.loads((folder/'metadata_original.json').read_text())
    if metadata['run_id'] != physical['run_id']:
        raise ValueError('Run identity mismatch: '+str(folder))
    if metadata['result'] != 'COMPLETED_TIMED_SEGMENT':
        raise ValueError('An aborted run cannot enter the completed-pilot calibration.')
    d0 = float(physical['start_gap_m'])
    df = float(physical['final_gap_m'])
    if not (d0 > df >= 0):
        raise ValueError('Invalid physical endpoint measurements.')
    if abs(d0-metadata['physical_start_gap_m']) > 1e-9:
        raise ValueError('Measured start gap differs from recorder metadata.')
    with (folder/'raw_events.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    events = [(float(r['elapsed_s']), r['topic_or_event'], unpack(r),
               float(r['receipt_ros_s'])) for r in rows]
    begin = float(metadata['motion_start_elapsed_s'])
    end = float(metadata['stop_elapsed_s'])
    initial = next(d for _,topic,d,_ in events if topic == 'MOTION_START')
    origin = initial['initial_odom']
    theta = 2*math.atan2(origin['quaternion'][2], origin['quaternion'][3])
    x0,y0 = origin['position']
    odom = []
    for elapsed,topic,d,ros in events:
        if topic != '/rm/odom':
            continue
        pose = d['pose']['pose']
        dx,dy = pose['position']['x']-x0, pose['position']['y']-y0
        stamp = d['header']['stamp']
        age = ros-(stamp['sec']+stamp['nanosec']*1e-9)
        # Header stamp is SDK callback receipt, not hardware acquisition time.
        source_elapsed = elapsed-age
        odom.append(dict(receipt=elapsed, source=source_elapsed, age=age,
                         x=dx*math.cos(theta)+dy*math.sin(theta),
                         y=-dx*math.sin(theta)+dy*math.cos(theta),
                         body_twist_x= d['twist']['twist']['linear']['x']))
    final_odom = [o for o in odom if o['receipt'] >= end+1.0][-10:]
    final_tof = [d['data'][0] for t,topic,d,_ in events
                 if topic == '/rm/range' and t >= end+1.0][-10:]
    if len(final_odom) < 5 or len(final_tof) < 5:
        raise ValueError('Insufficient settled endpoint samples: '+str(folder))
    xo = st.median(o['x'] for o in final_odom)
    r0 = initial['initial_tof']['range']
    rf = st.median(final_tof)
    if r0-rf <= 0:
        raise ValueError('Nonpositive ToF travel.')
    physical_travel = d0-df
    driver_faults = []
    tag_states = []
    raw_velocity = []
    seen_sequences = set()
    for t,topic,d,_ in events:
        if not begin <= t < end:
            continue
        if topic == '/rm/driver_status' and d['data'] != 'READY':
            driver_faults.append(dict(elapsed_s=t, status=d['data']))
        elif topic == '/rm/tag_observation':
            tag_states.append(d['status'])
        elif topic == '/rm/odom_raw':
            velocity = d['channels'].get('velocity')
            if velocity and velocity['seq'] not in seen_sequences:
                seen_sequences.add(velocity['seq'])
                raw_velocity.append(velocity['values'][3])
    jumps = []
    for a,b in zip(events,events[1:]):
        step = (b[3]-a[3])-(b[0]-a[0])
        if abs(step) > .05:
            jumps.append(dict(elapsed_s=b[0], step_s=step,
                              during_motion=begin <= b[0] < end))
    middle = [o for o in odom if begin+.5 <= o['receipt'] < end-.2]
    comparison = dict(commanded_speed_m_s=metadata['speed'],
                      odom_middle_slope_m_s=slope([(o['source'],o['x']) for o in middle]),
                      slope_pose_sample_count=len(middle),
                      raw_sdk_body_vx=spread(raw_velocity) if raw_velocity else None,
                      physical_endpoint_average_m_s=physical_travel/(end-begin))
    record = dict(run_id=metadata['run_id'], result=metadata['result'],
                  physical_start_gap_m=d0, physical_final_gap_m=df,
                  measurement_resolution_m=physical['resolution_m'],
                  physical_travel_m=physical_travel,
                  odometry_travel_m=xo, odometry_error_m=xo-physical_travel,
                  tof_start_m=r0, tof_final_m=rf, tof_drop_m=r0-rf,
                  tof_raw_error_m=(r0-rf)-physical_travel,
                  tof_scale_ratio=physical_travel/(r0-rf),
                  commanded_speed_m_s=metadata['speed'],
                  requested_duration_s=metadata['duration'],
                  actual_motion_window_s=end-begin,
                  settled_odom_n=len(final_odom), settled_tof_n=len(final_tof),
                  driver_faults_during_motion=driver_faults,
                  tag_status_counts_during_motion={s:tag_states.count(s) for s in sorted(set(tag_states))},
                  clock_jumps=jumps, speed_comparison=comparison,
                  notes=physical.get('notes', []))
    # Align by estimated callback receipt time on a monotonic run timeline.
    # Do not extrapolate or bridge source gaps larger than 0.2 seconds.
    valid = sorted((o['source'],o['x']) for o in odom if -.05 <= o['age'] <= .4)
    ts = [x[0] for x in valid]
    aligned = []
    for t,topic,d,_ in events:
        if topic != '/rm/range' or not begin <= t <= end+1.5:
            continue
        source = t-d['data'][1]
        j = bisect.bisect_left(ts,source)
        if not 0 < j < len(valid):
            continue
        ta,xa = valid[j-1]; tb,xb = valid[j]
        if not 0 < tb-ta <= .2:
            continue
        x = xa+(xb-xa)*(source-ta)/(tb-ta)
        aligned.append(dict(run_id=metadata['run_id'],time_from_motion_s=source-begin,
                            odom_progress_m=x,raw_tof_travel_m=r0-d['data'][0]))
    return record, aligned


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument('--output', type=Path, default=Path('results_local'))
    args = parser.parse_args()
    physical = json.loads((args.root/'physical_measurements.json').read_text())
    records, aligned = [],[]
    for item in physical['runs']:
        record, pairs = analyze_run(args.root/'runs'/item['run_id'],item)
        records.append(record); aligned.extend(pairs)
    if len(records) < 5:
        raise ValueError('At least five completed pilots required.')
    bo = st.median(r['odometry_error_m'] for r in records)
    scale = st.median(r['tof_scale_ratio'] for r in records)
    eo = [r['odometry_error_m']-bo for r in records]
    et = [scale*r['tof_drop_m']-r['physical_travel_m'] for r in records]
    summary = dict(stage='offline_pilot_calibration', evaluation_ready=False,
        pilot_count=len(records), odometry_bias_m=bo, tof_travel_scale=scale,
        bias_estimator='Median of settled odometry travel minus physical travel; all five pilots retained.',
        scale_estimator='Median of physical travel / raw ToF decrease; all five pilots retained.',
        endpoint_policy='MOTION_START sample for origin/r0; median last 10 odometry and ToF samples at least 1 s after STOP.',
        corrected_odometry_endpoint_spread=spread(eo),
        corrected_tof_endpoint_spread=spread(et),
        measurement_resolution_m=0.001,
        corrections=dict(odometry='z_o = x_raw - odometry_bias_m',
                         tof='z_T = tof_travel_scale * (r0 - r)'),
        limitations=[
            'Corrections fitted on five calibration pilots, not validated on evaluation data.',
            'Constant odometry bias is an endpoint approximation; applying it early in a run can create negative corrected progress.',
            'Endpoint residual spreads include motion-dependent errors and physical measurement error; they are not identified per-sample R.',
            'Full five-state process Q and measurement R are not determined by endpoint data alone; offline model checks and documented assumptions remain.',
            'Repeated ToF snapshots are not independent sensor samples; all ToF travel values share their run starting-range error.',
            'Camera pose and image transport changed before these five pilots; earlier slow02 is excluded from this set.',
            'slow05 brief driver interruption is retained; fast07 clock jump was before motion.',
            '1 mm tape division is resolution, not a guarantee of 1 mm accuracy.',
            'These pilots stop around 0.7-0.9 m. The final approach to 0.20 m and conservative raw ToF guard still require verification.'
        ], runs=records)
    clean = [r for r in records if not r['driver_faults_during_motion']]
    summary['sensitivity_excluding_driver_interruption'] = dict(
        n=len(clean), odometry_bias_m=st.median(r['odometry_error_m'] for r in clean),
        tof_travel_scale=st.median(r['tof_scale_ratio'] for r in clean),
        primary_analysis_retains_all_runs=True)
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'calibration_summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    columns=['run_id','physical_start_gap_m','physical_final_gap_m','physical_travel_m',
             'odometry_travel_m','odometry_error_m','tof_drop_m','tof_raw_error_m',
             'tof_scale_ratio','actual_motion_window_s','settled_odom_n','settled_tof_n']
    with (args.output/'calibration_table.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,columns,extrasaction='ignore');writer.writeheader();writer.writerows(records)
    with (args.output/'aligned_sensor_travel.csv').open('w',newline='') as f:
        fields=['run_id','time_from_motion_s','odom_progress_m','raw_tof_travel_m','scaled_tof_travel_m']
        writer=csv.DictWriter(f,fields);writer.writeheader()
        for row in aligned:
            writer.writerow(dict(row,scaled_tof_travel_m=scale*row['raw_tof_travel_m']))
    manifest={str(p.relative_to(args.root)):hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sorted((args.root/'runs').rglob('*')) if p.is_file()}
    manifest['physical_measurements.json']=hashlib.sha256((args.root/'physical_measurements.json').read_bytes()).hexdigest()
    (args.output/'input_sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('Completed pilots:',len(records))
    print(f'Odometry bias: {bo:.9f} m')
    print(f'ToF travel scale: {scale:.9f}')
    print('Offline calibration only: EKF/controller preview remains before evaluation.')
    print('Results:',args.output.resolve())


if __name__ == '__main__':
    main()
