from pathlib import Path
import sys,json,csv
import numpy as np
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'tmp/dm_sim_libs'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
E=R/'evidence/p2-arm/2026-11-01/dm-simulation';O=R/'output/dm-simulation-20261101'
rows=[];fig,axs=plt.subplots(1,2,figsize=(11,4))
for acc in [500,1000,2000,4000]:
 speed=300.;distance=45.;ta=min(speed/acc,np.sqrt(distance/acc));peak=acc*ta;tc=max(0,(distance-acc*ta*ta)/peak);total=2*ta+tc
 t=np.linspace(0,total,1001);q=np.zeros_like(t);v=np.zeros_like(t)
 for i,x in enumerate(t):
  if x<ta:q[i]=.5*acc*x*x;v[i]=acc*x
  elif x<ta+tc:q[i]=.5*acc*ta*ta+peak*(x-ta);v[i]=peak
  else:u=x-ta-tc;q[i]=.5*acc*ta*ta+peak*tc+peak*u-.5*acc*u*u;v[i]=max(0,peak-acc*u)
 assert abs(q[-1]-45)<1e-9 and max(v)<=300.00001
 rows.append({'hypothetical_acc_deg_s2':acc,'speed_cap_deg_s':300,'ideal_peak_deg_s':peak,'ideal_duration_s':total,'scope':'ideal kinematics, not identified drive parameters or motor simulation'})
 axs[0].plot(t,q,label=f'a={acc} deg/s^2');axs[1].plot(t,v,label=f'a={acc} deg/s^2')
for ax,y in zip(axs,['Angle (deg)','Speed (deg/s)']):ax.set_xlabel('Time (s)');ax.set_ylabel(y);ax.grid(alpha=.25);ax.legend(fontsize=8)
fig.suptitle('300 deg/s cap, 45 deg move: ideal profiles for ASSUMED acceleration values');fig.tight_layout();fig.savefig(O/'speed300-ideal-profiles.png',dpi=150)
(E/'speed300-ideal-profiles.json').write_text(json.dumps(rows,indent=2),encoding='utf-8');print(json.dumps(rows,indent=2))
