#!/usr/bin/env python3
"""List evaluation attempts and measured completed slots; no ROS or commands."""
import argparse
import json
from pathlib import Path
from freeze_protocol import digest


def inspect(root,output):
    manifest=root/'freeze_manifest.json'
    if not manifest.is_file():raise ValueError('Freeze the protocol first.')
    freeze=json.loads(manifest.read_text());sha=digest(manifest)
    records=[]
    for path in sorted(output.glob('*/metadata.json')):
        d=json.loads(path.read_text())
        if not d.get('evaluation_run'):continue
        issue=None
        if d.get('frozen_manifest_sha256')!=sha:issue='DIFFERENT_FROZEN_MANIFEST'
        elif not (path.parent/'raw_events.csv').is_file():issue='MISSING_RAW_LOG'
        elif d.get('result')!='COMPLETED_EVALUATION':issue=d.get('result','UNKNOWN')
        elif d.get('physical_final_gap_m') is None or d.get('physical_measurement_resolution_m') is None:
            issue='NEEDS_PHYSICAL_ENDPOINT'
        elif d.get('clock_jump_detected'):issue='CLOCK_JUMP'
        records.append(dict(run_id=d['run_id'],method=d['method'],slot=d.get('planned_slot'),
                            status=issue or 'MEASURED_COMPLETED',stop=d.get('stop_reason'),
                            final_gap_m=d.get('physical_final_gap_m')))
    completed={r['slot'] for r in records if r['status']=='MEASURED_COMPLETED'}
    return freeze,records,completed


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root',type=Path,default=Path('feedback_runs'))
    args=parser.parse_args();root=Path(__file__).resolve().parent
    freeze,records,completed=inspect(root,args.output_root)
    for r in records:
        print(f"{r['run_id']}: {r['status']}; stop={r['stop']}; physical final gap={r['final_gap_m']}")
    print('Measured completed unfiltered:',sum(s.startswith('eval_u') for s in completed),'/ 5')
    print('Measured completed filtered:',sum(s.startswith('eval_f') for s in completed),'/ 5')
    for slot in freeze['planned_order']:
        if slot in completed:continue
        outstanding=[r for r in records if r['slot']==slot and r['status']=='NEEDS_PHYSICAL_ENDPOINT']
        if outstanding:
            print('Next: record the measured physical endpoint for',outstanding[0]['run_id']);break
        method='unfiltered' if slot.startswith('eval_u') else 'filtered'
        run=slot;retry=0
        while (args.output_root/run).exists():
            retry+=1;run=slot+f'_retry{retry:02d}'
        print('Next planned run:',run,'method:',method)
        print('Stop driver before reset; measure start gap again. Retain all aborts.');break
    else:print('All ten measured evaluation slots complete. Next: analyze the full raw logs.')


if __name__=='__main__':main()
