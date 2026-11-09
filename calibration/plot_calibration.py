"""Plot calibration evidence only; no continuous physical ground truth."""
import argparse
import csv
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results',type=Path)
    args=parser.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    summary=json.loads((args.results/'calibration_summary.json').read_text())
    with (args.results/'aligned_sensor_travel.csv').open() as f:
        rows=list(csv.DictReader(f))
    names=[r['run_id'] for r in summary['runs']]
    fig,axes=plt.subplots(1,2,figsize=(10,4.6),layout='constrained')
    for name in names:
        r=[x for x in rows if x['run_id']==name]
        xs=[float(x['odom_progress_m']) for x in r]
        for ax,key in zip(axes,['raw_tof_travel_m','scaled_tof_travel_m']):
            ax.plot(xs,[float(x[key]) for x in r],'.',ms=2.4,label=f'{name} (n={len(r)})')
    for ax,title in zip(axes,['Raw ToF travel','Scaled ToF travel (fit on these pilots)']):
        ax.plot([0,.55],[0,.55],'k--',lw=1,label='Equal sensor progress')
        ax.set(xlabel='Raw odometry progress (m)',ylabel='ToF travel (m)',title=title)
        ax.grid(alpha=.25);ax.legend(fontsize=7)
    fig.suptitle('Five calibration pilots: aligned sensor observations, not physical trajectories',fontsize=11)
    fig.savefig(args.results/'sensor_travel.png',dpi=180);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,4.5),layout='constrained')
    offset=[-.23,0,.23]
    for shift,key,label in zip(offset,['odometry_error_m','corrected_odom','corrected_tof'],['Raw odometry','Bias-corrected odometry','Scaled ToF']):
        values=[]
        for r in summary['runs']:
            if key=='corrected_odom':value=r['odometry_error_m']-summary['odometry_bias_m']
            elif key=='corrected_tof':value=summary['tof_travel_scale']*r['tof_drop_m']-r['physical_travel_m']
            else:value=r[key]
            values.append(100*value)
        ax.scatter([i+shift for i in range(len(names))],values,label=label,s=42)
    ax.axhline(0,color='k',lw=1)
    ax.set(xticks=range(len(names)),xticklabels=names,ylabel='Estimated travel − physical travel (cm)',title='Calibration endpoint residuals: in-sample fit, not evaluation')
    ax.grid(axis='y',alpha=.25);ax.legend(fontsize=8)
    fig.savefig(args.results/'calibration_endpoint_residuals.png',dpi=180);plt.close(fig)
    print('Saved calibration figures in',args.results.resolve())


if __name__=='__main__':
    main()
