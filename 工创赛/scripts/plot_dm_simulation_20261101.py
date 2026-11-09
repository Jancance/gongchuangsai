from pathlib import Path
import json,csv,sys
import numpy as np
R=Path(__file__).resolve().parents[1];E=R/'evidence/p2-arm/2026-11-01/dm-simulation';O=R/'output/dm-simulation-20261101';O.mkdir(parents=True,exist_ok=True)
methods=['MIT_VMAX30','MIT_VMAX1','MIT_unquantized','Generic_cascade_PI'];DEG=180/np.pi
traces={m:np.loadtxt(E/(m+'-nominal-trace.csv'),delimiter=',',skiprows=1) for m in methods}
with (E/'metrics.csv').open(encoding='utf-8-sig') as f:rows=list(csv.DictReader(f))
for row in rows:
 for name in ['final_error_deg','speed_ripple_5to80hz_rms_deg_s']:row[name]=float(row[name])
# 标准绘图库，所有标题使用英文避免本机字体缺失。
sys.path.insert(0,str(R/'tmp/dm_sim_libs'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axs=plt.subplots(3,2,figsize=(14,12));colors=['#d97706','#2563eb','#64748b','#059669']
for method,color in zip(methods,colors):
 d=traces[method];tt=d[:,0];axs[0,0].plot(tt,d[:,1]*DEG,label=method,color=color,lw=1.3);axs[0,1].plot(tt,(d[:,1]-d[:,4])*DEG,color=color,lw=1.1,label=method)
 zoom=(tt>=6)&(tt<=7);axs[1,0].plot(tt[zoom],d[zoom,2]*DEG,color=color,lw=1,label=method)
 if method!='Generic_cascade_PI':axs[1,1].step(tt[zoom],d[zoom,7]*DEG,where='post',color=color,lw=1,label=method)
 rr=[x for x in rows if x['method']==method];offset=methods.index(method)*.18
 axs[2,0].scatter(np.arange(len(rr))+offset,[abs(x['final_error_deg']) for x in rr],s=13,color=color,label=method)
 axs[2,1].scatter(np.arange(len(rr))+offset,[x['speed_ripple_5to80hz_rms_deg_s'] for x in rr],s=13,color=color,label=method)
for ax,title,y in zip(axs.flat,['Example angle','Tracking error vs own reference','Example actual speed (zoom)','MIT decoded velocity command (zoom)','Sensitivity: terminal hold error','Sensitivity: speed ripple (5-80 Hz)'],['deg','deg','deg/s','deg/s','abs deg','RMS deg/s']):ax.set_title(title);ax.set_ylabel(y);ax.grid(alpha=.22)
for ax in axs[:2,:].flat:ax.set_xlabel('Time (s)')
for ax in axs[2,:].flat:ax.set_xlabel('Hypothetical plant / communication case')
axs[0,0].legend(fontsize=8);axs[1,1].legend(fontsize=8)
fig.suptitle('DM low-speed simulation - uncalibrated assumptions, NOT hardware validation',fontsize=14);fig.text(.5,.012,'45 deg / 15 s; same plant/torque limits. Generic cascade uses assumed PI gains + trapezoid; not vendor firmware reproduction.',ha='center',fontsize=9);fig.tight_layout(rect=[0,.03,1,.97]);fig.savefig(O/'comparison.png',dpi=155);plt.close(fig)
print('Artifacts:',O,flush=True)
