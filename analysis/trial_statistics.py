"""All-trial scatter plots and sample variance/covariance. Offline only."""
from pathlib import Path
import csv,json,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'analysis/results'
rows=list(csv.DictReader((OUT/'evaluation_trial_table.csv').open()))
assert len(rows)==10 and all(sum(r['method']==m for r in rows)==5 for m in ['unfiltered','filtered'])
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'savefig.bbox':'tight','figure.dpi':170})
keys=['raw_odom_progress_m_error_m','corrected_odom_progress_m_error_m','filtered_progress_m_error_m']
names=['Raw odometry','Corrected odometry','EKF']
def save(fig,name):
    for ext in ['png','pdf']:fig.savefig(OUT/(name+'.'+ext))
    plt.close(fig)
def csvwrite(path,rr):
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rr[0]));w.writeheader();w.writerows(rr)

# Every active recorded sample of every trial is shown; no fabricated trajectories.
fig,ax=plt.subplots(figsize=(8.7,5.2));counts={};data=[]
for i,r in enumerate(rows):
    rid=r['run_id'];method=r['method']
    ss=list(csv.DictReader((OUT/(rid+'_filter_states.csv')).open()))
    ss=[s for s in ss if s['phase']=='MOVING']
    key='raw_odom_progress_m' if method=='unfiltered' else 'filtered_progress_m'
    t=np.array([float(s['motion_elapsed_s']) for s in ss]);x=np.array([float(s[key]) for s in ss]);counts[rid]=len(ss)
    color=plt.get_cmap('tab10')(i)
    ax.scatter(t,x,s=11,alpha=.68,color=color,marker='o' if method=='unfiltered' else '^',label=rid.replace('eval_',''))
    ax.scatter(t[-1],x[-1],s=58,facecolors='none',edgecolors=[color],marker='o' if method=='unfiltered' else '^',linewidths=1.2)
    data.extend(dict(run_id=rid,method=method,motion_elapsed_s=float(ti),controlling_estimate_m=float(xi)) for ti,xi in zip(t,x))
t=np.linspace(0,6.1,250);tau=np.clip(t/4,0,1);ref=.4*(3*tau*tau-2*tau**3)
ax.plot(t,ref,'k--',lw=1.4,label='Common reference')
ax.axvline(4,c='gray',ls=':',lw=1)
ax.set(xlabel='Time from motion start (s)',ylabel='Controlling progress estimate (m)',title='All ten evaluation trials on one set of axes',xlim=(-.05,6.1),ylim=(-.005,.425))
ax.legend(title='Run (circles U, triangles F)',ncol=2,loc='upper left',fontsize=8,title_fontsize=9)
ax.grid(alpha=.18);fig.tight_layout();save(fig,'figure_E_all_trial_scatter')
csvwrite(OUT/'all_trial_scatter_samples.csv',data)

fig,ax=plt.subplots(figsize=(8.7,4.7));orders=np.arange(1,11);mean={}
for method,color,marker in [('unfiltered','#c66b20','o'),('filtered','#087f8c','^')]:
    ii=[i for i,r in enumerate(rows) if r['method']==method];gap=np.array([float(rows[i]['final_gap_m'])*100 for i in ii]);mean[method]=float(gap.mean())
    ax.scatter(orders[ii],gap,s=75,c=color,marker=marker,label=f'{method.capitalize()} trials, n=5',zorder=4)
    ax.axhline(gap.mean(),c=color,ls=':',lw=1.2,label=f'{method.capitalize()} mean {gap.mean():.2f} cm')
    for i,g in zip(ii,gap):ax.annotate(f'{g:.1f}',(orders[i],g),xytext=(0,-16 if i==2 else 8),textcoords='offset points',ha='center',fontsize=9,color=color)
ax.axhline(20,c='#a72339',ls='--',lw=1.5,label='Physical goal 20.00 cm')
ax.set(xticks=orders,xticklabels=[r['run_id'].replace('eval_','') for r in rows],xlabel='Evaluation run in actual execution order',ylabel='Measured final front clearance (cm)',title='Physical endpoint scatter for all ten evaluation trials',ylim=(19.3,28.7))
ax.legend(loc='lower center',bbox_to_anchor=(.5,.13),ncol=2,fontsize=8);ax.grid(axis='y',alpha=.18);fig.tight_layout();save(fig,'figure_F_endpoint_scatter')

variance=[];covariances={};correlations={};worked={}
for method in ['unfiltered','filtered']:
    q=[r for r in rows if r['method']==method];a=np.array([[float(r[k])*1000 for k in keys] for r in q]);C=np.cov(a,rowvar=False,ddof=1)
    # Verify against the scalar sample formula rather than only one library call.
    centered=a-a.mean(axis=0);manual=centered.T@centered/(len(a)-1)
    assert np.allclose(C,manual) and np.linalg.eigvalsh(C).min()>-1e-10
    assert np.isclose(C[0,0],C[1,1]) and np.isclose(C[0,1],C[0,0])
    covariances[method]=C.tolist();correlations[method]=np.corrcoef(a,rowvar=False).tolist()
    quantities=[('Physical final gap',np.array([float(r['final_gap_m'])*1000 for r in q]))]+[(name,a[:,j]) for j,name in enumerate(names)]
    for label,values in quantities:
        variance.append(dict(method=method,quantity=label if label=='Physical final gap' else label+' endpoint error',n=len(q),mean_mm=float(values.mean()),sample_sd_mm=float(values.std(ddof=1)),sample_variance_mm2=float(values.var(ddof=1))))
    csvwrite(OUT/(method+'_endpoint_error_covariance_mm2.csv'),[dict(estimator=names[i],**{names[j]:float(C[i,j]) for j in range(3)}) for i in range(3)])
    worked[method]=dict(raw_error_squared_deviation_sum_mm2=float((centered[:,0]**2).sum()),raw_EKF_cross_product_sum_mm2=float((centered[:,0]*centered[:,2]).sum()))
csvwrite(OUT/'sample_variance_by_method.csv',variance)
result=dict(sample_definition='One settled endpoint per independent trial; n=5 within each method. No pooling of periodic state samples.',denominator='n-1',covariance_units='mm^2',covariance_order=names,covariance_variables='estimator settled travel minus physically measured travel',variance_table=variance,covariance=covariances,correlation=correlations,worked_example=worked,scatter_sample_counts=counts,total_scatter_samples=sum(counts.values()),original_source='evaluation_trial_table.csv and original FEEDBACK_STATE extractions')
(OUT/'trial_statistics.json').write_text(json.dumps(result,indent=2))

def table(head,rr,fmt):
    return '\\begin{tabular}{'+fmt+'}\n\\toprule\n'+' & '.join(head)+r'\\\midrule'+'\n'+'\n'.join(' & '.join(map(str,r))+r'\\' for r in rr)+'\n\\bottomrule\n\\end{tabular}'
vtable=table(['Method','Quantity','$n$','Mean (mm)','SD (mm)','Variance (mm$^2$)'],[[v['method'][0].upper(),v['quantity'],v['n'],f"{v['mean_mm']:.3f}",f"{v['sample_sd_mm']:.3f}",f"{v['sample_variance_mm2']:.3f}"] for v in variance],'llrrrr')
section=r'''\subsection{All trials shown together}
To make it easier to compare individual trials, I plotted all ten evaluation runs on one set of axes. Each color identifies one run. Circles show unfiltered feedback and triangles show filtered feedback. I aligned time to motion start and included every recorded active feedback-state sample, without extending a run beyond its actual motion window. There are SAMPLECOUNT plotted samples in total; the number of independent trials is still ten.
\fig{figure_E_all_trial_scatter}{1.0}{Scatter plot of all ten evaluation trajectories, five U and five F. The vertical axis shows the estimate used by each controller: raw odometry for U and EKF progress for F. Hollow markers identify the last recorded active sample of each run. These are estimated trajectories, not continuous physical measurements. The dashed line is the common cubic reference and the vertical dotted line marks $T=4$ s. Every trial is retained.}
The traces follow a similar overall pattern, but their agreement does not show that the robot reached the physical target. For that reason, I also plotted all ten measured final gaps in their actual execution order.
\fig{figure_F_endpoint_scatter}{1.0}{Scatter plot of the measured physical final clearance in every evaluation run. Point labels are in cm. The method means are 25.50 cm (U) and 26.06 cm (F), and the physical goal is 20.00 cm. All ten runs stopped at the common raw ToF guard. The ruler division was 1 mm; no error bars are shown because this division is not a calibrated uncertainty bound.}

\subsection{Sample variance and covariance across trials}
I used one settled endpoint from each trial for the statistical analysis. This gives five observations per method, rather than treating the many samples within a run as independent trials. For a measured quantity $q_i$, I calculated the sample mean and variance as
\[
\bar q=\frac{1}{n}\sum_{i=1}^{n}q_i,\qquad
s_q^2=\frac{1}{n-1}\sum_{i=1}^{n}(q_i-\bar q)^2,\qquad s_q=\sqrt{s_q^2}.
\]
For two quantities measured on the same trials, the sample covariance is
\[
s_{ab}=\frac{1}{n-1}\sum_{i=1}^{n}(a_i-\bar a)(b_i-\bar b).
\]
Variance measures how much the results differ from their own mean. Covariance indicates whether two quantities tend to increase or decrease together. Both have squared units here, mm$^2$, because the input quantities are in mm. I used the $n-1$ denominator for both, with $n=5$ in each method group.
\begin{table}[H]\centering\small
VARIANCETABLE
\caption{Sample spread across the five trials in each method. For the physical gap row, the mean is the actual final clearance. For the three estimator rows, the mean is the signed travel error relative to measured physical travel. Their meanings differ, although both use mm. Variance of final clearance also equals variance of final-gap error because subtracting the constant 200 mm goal does not change variance.}
\end{table}
The final-gap variance was 50.000 mm$^2$ for unfiltered feedback and 36.800 mm$^2$ for filtered feedback. The corresponding SDs were 7.071 and 6.066 mm. Filtered runs were slightly more consistent in final gap, but their mean gap was farther from the 200 mm goal. A smaller variance therefore does not, on its own, mean better accuracy.

I formed a covariance matrix from the three endpoint estimation errors within each method. For each run the error vector was $\mathbf e_i=[e_{\mathrm{raw},i},e_{\mathrm{corrected},i},e_{\mathrm{EKF},i}]^\top$, and I used
\[
\bar{\mathbf e}=\frac{1}{n}\sum_i\mathbf e_i,\qquad
S_e=\frac{1}{n-1}\sum_i(\mathbf e_i-\bar{\mathbf e})(\mathbf e_i-\bar{\mathbf e})^\top.
\]
All entries in the following tables are in mm$^2$. The diagonal contains the individual error variances. The off-diagonal entries describe how the errors vary together across the same five runs. I calculated the methods separately; I did not treat an unfiltered run and the next filtered run as a matched measurement pair.
COVTABLES
For example, the filtered raw-error squared deviations sum to RAWSS mm$^2$. Dividing by $5-1=4$ gives a sample variance of RAWVAR mm$^2$. The raw/EKF centered cross-products sum to CROSSSUM mm$^2$, giving covariance CROSSCOV mm$^2$ with the same denominator. These calculations use the full-precision estimates, so recalculating from the rounded table entries can give slightly different results. Extra decimal places describe arithmetic precision, not physical measurement accuracy.

The covariance of raw and corrected odometry equals their variance in both groups. This is expected: at these settled endpoints, corrected odometry is raw odometry minus the same 6.774102 mm offset. Subtracting a constant changes the mean error but leaves variance unchanged, and the correlation of the two error vectors is exactly 1. The covariance matrices are therefore singular. This is a mathematical consequence of the correction, not evidence of an extra independent sensor.

The raw/EKF error correlations were 0.935 for U and 0.917 for F. Their positive covariance is consistent with the EKF using odometry, and all three errors also share the same measured physical travel in their definition. Measurement error in that physical travel can itself introduce covariance. These empirical matrices describe the spread of endpoint errors across trials; they are different from the EKF's internal five-state covariance $P$ and do not identify a sensor-noise matrix $R$. With only five trials per method, I use them as descriptive statistics rather than proof of an improvement or a general noise model.
'''
covtex=[]
for method in ['unfiltered','filtered']:
    C=np.array(covariances[method]);tab=table(['Error','Raw','Corrected','EKF'],[[names[i]]+[f'{C[i,j]:.3f}' for j in range(3)] for i in range(3)],'lrrr')
    covtex.append('\\begin{table}[H]\\centering\n'+tab+'\n\\caption{'+method.capitalize()+' feedback endpoint-error sample covariance, five trials, all entries in mm$^2$. Rows and columns refer to the same run-level error variables.}\n\\end{table}')
f=worked['filtered'];C=np.array(covariances['filtered'])
for key,val in {'SAMPLECOUNT':str(sum(counts.values())),'VARIANCETABLE':vtable,'COVTABLES':'\n'.join(covtex),'RAWSS':f"{f['raw_error_squared_deviation_sum_mm2']:.6f}",'RAWVAR':f'{C[0,0]:.6f}','CROSSSUM':f"{f['raw_EKF_cross_product_sum_mm2']:.6f}",'CROSSCOV':f'{C[0,2]:.6f}'}.items():section=section.replace(key,val)
(OUT/'trial_statistics_section.tex').write_text(section)
print(json.dumps(result,indent=2))
