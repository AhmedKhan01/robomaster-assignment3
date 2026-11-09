"""Offline audit and four assignment figures. Never imports ROS or sends commands.
Run from any directory: python3 analysis/analyze_evaluation.py
"""
from pathlib import Path
import csv, json, hashlib, collections
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
E=ROOT/'evaluation'; OUT=ROOT/'analysis'/'results'; OUT.mkdir(exist_ok=True)
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'figure.dpi':150,'savefig.bbox':'tight'})
COLORS=['#64748b','#d97706','#087f8c']
KEYS=['raw_odom_progress_m','corrected_odom_progress_m','filtered_progress_m']
LABELS=['Raw odometry','Bias-corrected odometry','EKF']

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def stats(a):
    a=np.asarray(a,float)
    return dict(n=len(a),mean=float(a.mean()),sample_sd=float(a.std(ddof=1)) if len(a)>1 else None,
                MAE=float(abs(a).mean()),RMSE=float(np.sqrt(np.mean(a*a))),minimum=float(a.min()),maximum=float(a.max()))
def writecsv(path,rows):
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def save(fig,name):
    for ext in ['pdf','png']:fig.savefig(OUT/(name+'.'+ext))
    plt.close(fig)
def unpack(s):
    p=json.loads(s)
    if isinstance(p,dict) and isinstance(p.get('data'),str):
        try:p=json.loads(p['data'].replace('\\"','"'))
        except json.JSONDecodeError:pass
    return p

manifest=json.loads((E/'freeze_manifest.json').read_text()); frozen=digest(E/'freeze_manifest.json')
checks=[]
for group in ['package_sha256','snapshot_sha256']:
    for rel,h in manifest[group].items():
        actual=digest(E/rel); checks.append(dict(file=rel,expected=h,actual=actual,match=actual==h))
assert all(x['match'] for x in checks),'Frozen file changed'
runs=[];table=[];stoprows=[];source_hashes={}
for runid in manifest['planned_order']:
    p=E/'feedback_runs'/runid;m=json.loads((p/'metadata.json').read_text())
    assert m['frozen_manifest_sha256']==frozen
    assert m['result']=='COMPLETED_EVALUATION' and m['physical_final_gap_m'] is not None
    assert m['protocol']==json.loads((E/'protocol.json').read_text())
    assert m['ekf_config']==json.loads((E/'ekf_config.json').read_text())
    assert all(digest(E/k)==v for k,v in m['source_sha256'].items())
    source_hashes[str((p/'raw_events.csv').relative_to(ROOT))]=digest(p/'raw_events.csv')
    events=[]
    with (p/'raw_events.csv').open() as f:
        for r in csv.DictReader(f):events.append(dict(t=float(r['elapsed_s']),ros=float(r['receipt_ros_s']),topic=r['topic_or_event'],p=unpack(r['payload_json'])))
    states=[dict(t=r['t'],**r['p']) for r in events if r['topic']=='FEEDBACK_STATE' and r['p'].get('filtered_progress_m') is not None]
    start=m['motion_start_elapsed_s'];stop=m['stop_elapsed_s'];duration=stop-start
    settled=[s for s in states if s['t']>=stop+1.][-10:]
    assert len(settled)>=8,'Insufficient settled window'
    D=m['physical_start_gap_m']-m['physical_final_gap_m']
    row=dict(run_id=runid,method=m['method'],start_gap_m=m['physical_start_gap_m'],final_gap_m=m['physical_final_gap_m'],physical_travel_m=D,
             final_gap_error_m=m['physical_final_gap_m']-.2,motion_duration_s=duration,stop_reason=m['stop_reason'],settled_state_n=len(settled))
    for k in KEYS+['scaled_tof_travel_m']:
        row[k]=float(np.median([s[k] for s in settled]));row[k+'_error_m']=row[k]-D
    table.append(row)
    moving=[s for s in states if s['phase']=='MOVING']
    assert all(s['status']=='HEALTHY' for s in moving)
    assert all(0<=s['requested_v_m_s']<=.16000001 and abs(s['requested_omega_rad_s'])<=.10000001 for s in states)
    assert not m['clock_jump_detected']
    for s in states:
        P=np.asarray(s['covariance']);assert np.isfinite(P).all() and np.linalg.eigvalsh(P).min()>-1e-10
    starts=[r for r in events if r['topic']=='MOTION_START'];assert len(starts)==1
    initial=starts[0]['p'];r0=initial['initial_tof']['range']
    # Offline alignment on the common callback-receipt source-time axis.
    # Range source time is receipt elapsed minus driver-reported sensor age.
    # Interpolate odometry pose and logged EKF state at that same time, never extrapolate.
    origin=initial['initial_odom'];q=origin['quaternion']
    yaw=np.arctan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
    poses={}
    for r in events:
        if r['topic']!='/rm/odom':continue
        d=r['p'];stamp=d['header']['stamp'];source=r['t']-(r['ros']-stamp['sec']-stamp['nanosec']*1e-9)
        xy=d['pose']['pose']['position'];dx=xy['x']-origin['position'][0];dy=xy['y']-origin['position'][1]
        poses[source]=dx*np.cos(yaw)+dy*np.sin(yaw)
    ot=np.array(sorted(poses));ox=np.array([poses[t] for t in ot]);ts=np.array([s['t'] for s in states]);fx=np.array([s[KEYS[2]] for s in states])
    residuals=[]
    for r in events:
        if r['topic']!='/rm/range' or not start<=r['t']<=stop:continue
        a=r['p'].get('data',[])
        if not isinstance(a,list) or len(a)<2:continue
        source=r['t']-a[1];j=np.searchsorted(ts,source,side='right');i=np.searchsorted(ot,source,side='right')
        if not (0<j<len(ts) and 0<i<len(ot)) or not 0<=a[1]<=.4:continue
        if ts[j]-ts[j-1]>.10 or ot[i]-ot[i-1]>.20 or source<start:continue
        raw=float(np.interp(source,ot,ox));fil=float(np.interp(source,ts,fx));travel=r0-a[0]
        residuals.append(dict(run_id=runid,t_s=source-start,range_source_age_s=a[1],odom_bracket_s=ot[i]-ot[i-1],filter_bracket_s=ts[j]-ts[j-1],raw_tof_travel_m=travel,
                              raw_odom_m=raw,filtered_m=fil,raw_residual_m=travel-raw,filtered_residual_m=travel-fil))
    writecsv(OUT/(runid+'_aligned_residuals.csv'),residuals)
    writecsv(OUT/(runid+'_filter_states.csv'),[dict(elapsed_s=s['t'],motion_elapsed_s=s['motion_elapsed_s'],phase=s['phase'],status=s['status'],
              reference_x_m=s['reference_x_m'],requested_v_m_s=s['requested_v_m_s'],requested_omega_rad_s=s['requested_omega_rad_s'],
              **{k:s[k] for k in KEYS},scaled_tof_travel_m=s['scaled_tof_travel_m'],state_json=json.dumps(s['ekf_state']),covariance_json=json.dumps(s['covariance'])) for s in states])
    for r in events:
        if r['topic'] in ['STOP','CLOCK_JUMP','STATE_TRANSITION']:stoprows.append(dict(run_id=runid,elapsed_s=r['t'],event=r['topic'],payload_json=json.dumps(r['p'])))
    runs.append(dict(id=runid,m=m,states=states,events=events,residuals=residuals,row=row,duration=duration))
assert [r['m']['created_utc'] for r in runs]==sorted(r['m']['created_utc'] for r in runs)
allmetas=[json.loads(p.read_text()) for p in (E/'feedback_runs').glob('*/metadata.json')]
attempts=[m for m in allmetas if m.get('evaluation_run')]
audit=dict(frozen_manifest_sha256=frozen,frozen_files_checked=len(checks),hash_checks=checks,raw_log_sha256=source_hashes,
           executed_evaluation_attempts=len(attempts),completed=len(runs),aborted=sum(m.get('result')=='ABORTED' for m in attempts),
           previews=sum(not m.get('execute') for m in allmetas),order_verified=True,all_active_state_checks_healthy=True,
           all_covariances_positive_semidefinite=True,physical_placement_verified_by_hash=False,
           endpoint_policy='Median last ten logged FEEDBACK_STATE values at least 1 second after STOP; time samples are correlated.',
           residual_policy='Range source time = receipt elapsed minus source age. Interpolate odometry pose and logged EKF state on that same callback-receipt timeline, max bracket 0.20/0.10 s, no extrapolation, range age <=0.40 s. Hardware acquisition synchronization unavailable.')
(OUT/'audit.json').write_text(json.dumps(audit,indent=2))
writecsv(OUT/'evaluation_trial_table.csv',table);writecsv(OUT/'stop_events.csv',stoprows)
metrics={}
for method in ['unfiltered','filtered']:
    tt=[r for r in table if r['method']==method]
    metrics[method]={k:stats([r[k] for r in tt]) for k in ['final_gap_error_m']+[k+'_error_m' for k in KEYS]}
filtered=[r for r in runs if r['m']['method']=='filtered']
# f02 is the largest physical final gap, a sensitivity case, never invalidated.
metrics['filtered_without_largest_gap_f02']={k:stats([r['row'][k] for r in filtered if r['id']!='eval_f02']) for k in [k+'_error_m' for k in KEYS]}
metrics['leave_one_out_filtered_RMSE_ranges']={k:[min(stats([r['row'][k+'_error_m'] for r in filtered if r['id']!=omit['id']])['RMSE'] for omit in filtered),max(stats([r['row'][k+'_error_m'] for r in filtered if r['id']!=omit['id']])['RMSE'] for omit in filtered)] for k in KEYS}
(OUT/'metrics.json').write_text(json.dumps(metrics,indent=2))

# A: first filtered run, declared selection independent of outcome.
r=filtered[0];ss=r['states'];t=np.array([s['motion_elapsed_s'] for s in ss]);fig,ax=plt.subplots(2,1,figsize=(8,6),sharex=True,gridspec_kw={'height_ratios':[2,1]})
ax[0].plot(t,[s['reference_x_m'] for s in ss],'k--',label='Cubic reference')
for k,l,c in zip(KEYS,LABELS,COLORS):ax[0].plot(t,[s[k] for s in ss],color=c,label=l,lw=1.7)
ax[0].scatter([t[-1]],[r['row']['physical_travel_m']],marker='x',s=55,c='black',label='Measured endpoint only')
ax[0].set(ylabel='Travel (m)',title='A  First filtered evaluation: eval_f01');ax[0].legend(ncol=2,fontsize=8)
ax[1].step(t,[s['requested_v_m_s'] for s in ss],where='post',color=COLORS[2],label='Requested forward speed')
ax[1].axhline(.06,color='gray',ls=':',label='Near-box cap')
for a in ax:
    a.axvline(4,color='#9f1239',ls='--',label='Reference end T' if a==ax[1] else None)
    a.axvline(r['duration'],color='#7c3aed',ls=':',label='Actual guard stop' if a==ax[1] else None);a.grid(alpha=.18)
ax[1].set(xlabel='Time from motion start (s)',ylabel='Speed (m/s)');ax[1].legend(fontsize=8,ncol=2)
fig.tight_layout();save(fig,'figure_A_representative')

# B: common active interval within each method; five independent runs, no extrapolation.
fig,ax=plt.subplots(figsize=(8,4.6));tracking=[]
for method,c in [('unfiltered','#c66b20'),('filtered','#087f8c')]:
    rr=[r for r in runs if r['m']['method']==method];end=min(max(s['motion_elapsed_s'] for s in r['states'] if s['phase']=='MOVING') for r in rr)
    begin=max(min(s['motion_elapsed_s'] for s in r['states'] if s['phase']=='MOVING') for r in rr)
    grid=np.arange(max(.05,begin),end+.00001,.05);yy=[]
    key=KEYS[0] if method=='unfiltered' else KEYS[2]
    for r in rr:
        ss=[s for s in r['states'] if s['phase']=='MOVING'];yy.append(np.interp(grid,[s['motion_elapsed_s'] for s in ss],[s['reference_x_m']-s[key] for s in ss]))
    yy=np.asarray(yy);mu=yy.mean(0)*100;sd=yy.std(0,ddof=1)*100
    ax.plot(grid,mu,c=c,label=method.capitalize()+' mean, n=5');ax.fill_between(grid,mu-sd,mu+sd,color=c,alpha=.16)
    tracking.extend(dict(method=method,time_s=float(t),mean_error_m=float(v/100),sample_sd_m=float(d/100),n=5) for t,v,d in zip(grid,mu,sd))
ax.axvline(4,c='#9f1239',ls='--',label='T = 4 s');ax.set(xlabel='Time from motion start (s)',ylabel='Estimated tracking error (cm)',title='B  Reference minus controlling progress estimate')
ax.legend(fontsize=9);ax.grid(alpha=.18);fig.tight_layout();save(fig,'figure_B_tracking');writecsv(OUT/'tracking_mean_sd.csv',tracking)

# C: identical five filtered trajectories for each estimator.
fig,axes=plt.subplots(1,2,figsize=(10,4),gridspec_kw={'width_ratios':[1.7,1]});xx=np.arange(5)
for i,(k,l,c) in enumerate(zip(KEYS,LABELS,COLORS)):
    ee=np.array([r['row'][k+'_error_m'] for r in filtered])*100
    axes[0].bar(xx+(i-1)*.24,ee,.23,color=c,label=l)
axes[0].axhline(0,c='black',lw=.7);axes[0].set(xticks=xx,xticklabels=['f01','f02','f03','f04','f05'],ylabel='Estimate minus physical travel (cm)',title='C  Endpoint validation on five filtered runs');axes[0].legend(fontsize=8)
for i,(k,l,c) in enumerate(zip(KEYS,LABELS,COLORS)):
    v=metrics['filtered'][k+'_error_m'];axes[1].bar(np.array([0,1])+(i-1)*.24,[v['MAE']*100,v['RMSE']*100],.23,color=c,label=l)
axes[1].set(xticks=[0,1],xticklabels=['MAE','RMSE'],ylabel='Endpoint error (cm)',title='All five retained')
for a in axes:a.grid(axis='y',alpha=.18)
fig.tight_layout();save(fig,'figure_C_endpoint')

# D: causal time-matched residuals sampled when raw odometry first crosses each progress point.
progress=np.arange(.05,.301,.05);resid=[];alignrows=[]
for r in filtered:
    samples=r['residuals'];pair=[]
    for x in progress:
        s=next(s for s in samples if s['raw_odom_m']>=x)
        pair.append([s['raw_residual_m'],s['filtered_residual_m']]);alignrows.append(dict(progress_m=float(x),**s))
    resid.append(pair)
resid=np.asarray(resid);fig,axes=plt.subplots(2,2,figsize=(10,8))
for j,label in enumerate(['Raw ToF travel - raw odometry','Raw ToF travel - EKF']):
    a=axes[0,j];mu=resid[:,:,j].mean(0)*100;sd=resid[:,:,j].std(0,ddof=1)*100
    a.plot(progress,mu,'o-',color=COLORS[j*2]);a.fill_between(progress,mu-sd,mu+sd,color=COLORS[j*2],alpha=.17)
    a.set(xlabel='Raw odometry progress (m)',ylabel='Residual (cm)',title=label+'\nMean +/- sample SD, n=5');a.grid(alpha=.18)
    C=np.corrcoef(resid[:,:,j],rowvar=False);np.savetxt(OUT/('residual_correlation_'+['raw','filtered'][j]+'.csv'),C,delimiter=',',header=','.join(f'{v:.2f}m' for v in progress),comments='')
    a=axes[1,j];im=a.imshow(C,vmin=-1,vmax=1,cmap='RdBu_r');a.set(xticks=range(6),yticks=range(6),xticklabels=[f'{v:.2f}' for v in progress],yticklabels=[f'{v:.2f}' for v in progress],xlabel='Progress point (m)',ylabel='Progress point (m)',title='Across-run correlation, n=5')
    for y in range(6):
        for x in range(6):a.text(x,y,f'{C[y,x]:.2f}',ha='center',va='center',fontsize=8,color='white' if abs(C[y,x])>.6 else 'black')
fig.subplots_adjust(right=.88); cax=fig.add_axes([.91,.13,.015,.29]);fig.colorbar(im,cax=cax,label='Pearson r');fig.suptitle('D  Raw ToF residual dependence on the same five filtered runs',y=1.01)
fig.subplots_adjust(hspace=.50,wspace=.35);save(fig,'figure_D_residuals');writecsv(OUT/'residual_progress_points.csv',alignrows)
print(json.dumps(dict(audit={k:v for k,v in audit.items() if k not in ['hash_checks','raw_log_sha256']},metrics=metrics),indent=2))
