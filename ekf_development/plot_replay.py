import argparse
import csv
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description='Plot calibration replay, not evaluation results.')
    parser.add_argument('replay_dir',type=Path);args=parser.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    results=json.loads((args.replay_dir/'replay_summary.json').read_text())['runs']
    fig,axes=plt.subplots(len(results),1,figsize=(9,12),layout='constrained')
    for ax,result in zip(axes,results):
        with (args.replay_dir/(result['run_id']+'_states.csv')).open() as f:rows=list(csv.DictReader(f))
        times=[float(x['time_from_motion_s']) for x in rows]
        for key,label in [('raw_odom_m','Raw odometry'),('scaled_tof_m','Scaled ToF'),('ekf_x_m','EKF prototype')]:
            ax.plot(times,[float(x[key]) for x in rows],label=label,lw=1.3)
        ax.scatter([times[-1]],[result['physical_travel_m']],marker='x',s=70,color='black',label='Measured settled physical travel')
        ax.set(title=result['run_id'],xlabel='Time since motion start (s)',ylabel='Progress (m)')
        ax.grid(alpha=.25);ax.legend(fontsize=7,loc='upper left')
    fig.suptitle('In-sample pilot replay: endpoint measurement only; no continuous physical ground truth',fontsize=11)
    fig.savefig(args.replay_dir/'pilot_replay.png',dpi=160)
    print('Saved',args.replay_dir/'pilot_replay.png')


if __name__=='__main__':main()
