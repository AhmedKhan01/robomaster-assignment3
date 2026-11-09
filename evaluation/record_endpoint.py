#!/usr/bin/env python3
"""Record a physically measured settled endpoint without changing raw logs."""
import argparse
import json
from pathlib import Path
import shutil
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id',required=True)
    parser.add_argument('--final-gap',required=True,type=float)
    parser.add_argument('--resolution',default=.001,type=float)
    parser.add_argument('--output-root',type=Path,default=Path('feedback_runs'))
    args=parser.parse_args()
    import math
    if Path(args.run_id).name!=args.run_id or args.run_id in ('.','..'):parser.error('Invalid run ID.')
    if not math.isfinite(args.final_gap) or not 0<=args.final_gap<=3:parser.error('Invalid gap.')
    if not math.isfinite(args.resolution) or not 0<args.resolution<=.1:parser.error('Invalid resolution.')
    p=args.output_root/args.run_id/'metadata.json';data=json.loads(p.read_text())
    if data['run_id']!=args.run_id:raise ValueError('Run identity mismatch.')
    shutil.copy2(p,p.with_name(f'metadata.backup_{time.time_ns()}.json'))
    data['physical_final_gap_m']=args.final_gap
    data['physical_measurement_resolution_m']=args.resolution
    p.write_text(json.dumps(data,indent=2)+'\n')
    print('Saved physical final gap:',args.final_gap,'m')
    print('Physical travel:',data['physical_start_gap_m']-args.final_gap,'m')
    print('Stage remains:',data['stage'])


if __name__=='__main__':main()
