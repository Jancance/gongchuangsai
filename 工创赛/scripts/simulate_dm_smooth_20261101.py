"""离线假设模型，非厂家固件复现或本机辨识；不含硬件接口。"""
from pathlib import Path
import sys,json,csv,math,itertools,hashlib
import numpy as np
from scipy.signal import butter,sosfiltfilt
R=Path(__file__).resolve().parents[1];E=R/'evidence/p2-arm/2026-11-01/dm-simulation';O=R/'output/dm-simulation-20261101';E.mkdir(parents=True,exist_ok=True);O.mkdir(parents=True,exist_ok=True)
DEG=180/np.pi;START=np.deg2rad(33.8182983);DIST=np.deg2rad(45);GOAL=START+DIST;T=15.;END=18.;DT=.0005
methods=['MIT_VMAX30','MIT_VMAX1','MIT_unquantized','Generic_cascade_PI']
cases=[dict(inertia=j,static_friction=fs,coulomb_friction=.75*fs,viscous_friction=.025,load=load,comm=comm) for j,fs,load,comm in itertools.product([.004,.012,.036],[.01,.04,.08],[0.,.02],['2ms','jitter_2to8ms'])]
J=np.array([c['inertia'] for c in cases]);FS=np.array([c['static_friction'] for c in cases]);FC=.75*FS;B=np.full(len(cases),.025);LOAD=np.array([c['load'] for c in cases]);N=len(cases)
def trajectory(t):
 s=np.clip(t/T,0,1);return START+DIST*s**3*(10+s*(-15+6*s)),DIST/T*30*s*s*(1-s)**2 if t<T else 0.
def trapezoid(t):
 # 同45度、15秒，假设驱动内部梯形；非厂家固件的已知具体实现。
 a=DIST/(3*12);vp=a*3
 if t<3:return START+.5*a*t*t,a*t
 if t<12:return START+.5*a*9+vp*(t-3),vp
 if t<15:
  u=t-12;return START+.5*a*9+vp*9+vp*u-.5*a*u*u,vp-a*u
 return GOAL,0.
def quant(x,span,bits):return np.floor(np.clip((x+span)/(2*span),0,1)*(2**bits-1))/(2**bits-1)*(2*span)-span

def simulate(method,dt=DT,indices=None,keep=True):
 ids=np.arange(N) if indices is None else np.asarray(indices);n=len(ids);j,fs,fc,b,load=J[ids],FS[ids],FC[ids],B[ids],LOAD[ids]
 q=np.full(n,START);w=np.zeros(n);tau=np.zeros(n);p_cmd=q.copy();v_cmd=w.copy();next_tx=np.zeros(n);iv=np.zeros(n);ip=np.zeros(n);kp=40*500/4095;kd=409*5/4095;ff=quant(0.,10,12)
 rng=np.random.default_rng(411);regular=np.array([cases[k]['comm']=='2ms' for k in ids]);records=[];maxspeed=np.zeros(n);maxtau=np.zeros(n);work=np.zeros(n)
 for k in range(int(round(END/dt))+1):
  t=k*dt;p_ref,v_ref=(trapezoid(t) if method=='Generic_cascade_PI' else trajectory(t));enc=np.round(q/(2*np.pi/16384))*(2*np.pi/16384)
  if method=='Generic_cascade_PI':
   pe=p_ref-enc;ip=np.clip(ip+pe*dt,-.15,.15);vr=np.clip(v_ref+6*pe+.5*ip,-.3,.3);ve=vr-w
   candidate=np.clip(iv+3*ve*dt,-.6,.6);raw=1.2*ve+candidate;allow=(np.abs(raw)<=1)|((raw>1)&(ve<0))|((raw< -1)&(ve>0));iv=np.where(allow,candidate,iv);target_tau=np.clip(1.2*ve+iv,-1,1)
  else:
   update=t+1e-10>=next_tx
   if np.any(update):
    span=30 if method=='MIT_VMAX30' else 1
    p_cmd[update]=p_ref if method=='MIT_unquantized' else quant(p_ref,12.5,16)
    v_cmd[update]=v_ref if method=='MIT_unquantized' else quant(v_ref,span,12)
    intervals=np.where(regular,.002,rng.uniform(.002,.008,n));next_tx[update]=t+intervals[update]
   # 驱动内部使用自身编码器/速度，不把CAN反馈12位误当内部速度环分辨率。
   target_tau=np.clip(kp*(p_cmd-enc)+kd*(v_cmd-w)+(0 if method=='MIT_unquantized' else ff),-1,1)
  # 假设一阶电流响应2ms，静/库仑/粘性摩擦；不是实测电机电气模型。
  tau+=(target_tau-tau)*min(1,dt/.002);net=tau-load-b*w;stationary=(np.abs(w)<1e-4)&(np.abs(net)<=fs)
  direction=np.where(np.abs(w)<1e-4,np.sign(net),np.sign(w));acc=(net-fc*direction)/j
  nw=w+acc*dt;cross=(w*nw<0)&(np.abs(net)<=fs);nw=np.where(stationary|cross,0,nw);q+=.5*(w+nw)*dt;w=nw
  maxspeed=np.maximum(maxspeed,np.abs(w));maxtau=np.maximum(maxtau,np.abs(tau));work+=tau*tau*dt
  if keep and k%max(1,round(.002/dt))==0:records.append(np.column_stack([np.full(n,t),q,w,tau,np.full(n,p_ref),np.full(n,v_ref),p_cmd,v_cmd]))
 traces=np.asarray(records) if keep else None
 return q,w,maxspeed,maxtau,work,traces
rows=[];traces={};trace_id=cases.index(dict(inertia=.012,static_friction=.04,coulomb_friction=.03,viscous_friction=.025,load=.02,comm='2ms'))
for method in methods:
 q,w,ms,mt,work,data=simulate(method);traces[method]=data[:,trace_id,:];times=data[:,0,0];active=(times>=3)&(times<=12);err=data[:,:,1]-data[:,:,4];verr=data[:,:,2]-data[:,:,5]
 sos=butter(3,[5,80],btype='bandpass',fs=500,output='sos');ripple=sosfiltfilt(sos,data[:,:,2],axis=0)
 for n,c in enumerate(cases):
  rows.append(dict(method=method,case=n,**c,final_error_deg=(q[n]-GOAL)*DEG,tracking_rmse_deg=float(np.sqrt(np.mean(err[:,n]**2))*DEG),velocity_tracking_rmse_deg_s=float(np.sqrt(np.mean(verr[active,n]**2))*DEG),speed_ripple_5to80hz_rms_deg_s=float(np.sqrt(np.mean(ripple[active,n]**2))*DEG),max_speed_rad_s=ms[n],max_torque_nm=mt[n],torque_rms_nm=math.sqrt(work[n]/END),speed_guard_would_trip=bool(ms[n]>.5),note='unidentified hypothetical plant; terminal hold enabled to18s, not firmware early-disable reproduction'))
 print(method,'done',flush=True)
with (E/'metrics.csv').open('w',newline='',encoding='utf-8-sig') as f:
 writer=csv.DictWriter(f,fieldnames=rows[0].keys());writer.writeheader();writer.writerows(rows)
summary={}
for method in methods:
 rr=[v for v in rows if v['method']==method];summary[method]={name:{'median':float(np.median([abs(x[name]) for x in rr])),'min':float(np.min([abs(x[name]) for x in rr])),'max':float(np.max([abs(x[name]) for x in rr]))} for name in ['final_error_deg','tracking_rmse_deg','speed_ripple_5to80hz_rms_deg_s','velocity_tracking_rmse_deg_s','max_torque_nm']};summary[method]['speed_guard_cases']=sum(x['speed_guard_would_trip'] for x in rr)
for method in methods:
 np.savetxt(E/(method+'-nominal-trace.csv'),traces[method],delimiter=',',header='time_s,position_rad,velocity_rad_s,torque_nm,reference_rad,reference_rad_s,held_can_position_rad,held_can_velocity_rad_s',comments='')
# 积分步长收敛核对，非实物辨识。
convergence={}
for method in methods:
 a=simulate(method,indices=[trace_id],keep=False);b=simulate(method,dt=DT/2,indices=[trace_id],keep=False);convergence[method]={'terminal_angle_difference_deg':float(abs(a[0][0]-b[0][0])*DEG),'max_speed_difference_rad_s':float(abs(a[2][0]-b[2][0]))}
assert all(np.isfinite(float(z['final_error_deg'])) and z['max_torque_nm']<=1.000001 for z in rows)
assert all(v['terminal_angle_difference_deg']<.05 for v in convergence.values()),convergence
assumptions={'explicit_scope':'Synthetic sensitivity analysis only; no identified motor/arm model, no real firmware cascade gains or motor firmware simulation','plant_cases':cases,'methods':methods,'integration_step_s':DT,'trace_sample_step_s':.002,'motion_deg':45,'motion_time_s':15,'simulation_time_s':18,'terminal_hold':'Controllers hold until18s; actual firmware may disable soon after reaching3deg tolerance. Model final errors are not actual stopped-position predictions.','driver_encoder_assumption':'14-bit single-turn output angle quantization; internal velocity assumed observable, not CAN12-bit velocity','current_response_assumption_s':.002,'torque_limit_nm':1,'mit_parameters':{'kp_encoded':40*500/4095,'kd_encoded':409*5/4095,'pmax':12.5,'vmax_baseline':30,'vmax_candidate':1,'tmax':10},'generic_cascade_assumptions':{'position_p':6,'position_i':.5,'velocity_p':1.2,'velocity_i':3,'trapezoid_accel_time_s':3,'velocity_cap_rad_s':.3,'not_reference_motor_gains':True},'limitations':['No backlash, cogging, actual gear compliance, temperature, supply or true velocity estimator model','Hypothetical friction/inertia/load ranges, not identified or fit to user observations','No enable/disable/timeout/trajectory state machine equivalence; firmware protections are not evaluated by these plant plots','No claim higher-fidelity mode beats MIT on this physical robot','Communication2ms or random2..8ms assumed, not measured','Reference-code300deg/s deliberately not copied; equal45deg/15s duration used'], 'reference_code_task_pending':False}
(E/'assumptions.json').write_text(json.dumps(assumptions,indent=2),encoding='utf-8');(E/'summary.json').write_text(json.dumps({'summary':summary,'integration_convergence':convergence,'nominal_case':trace_id},indent=2),encoding='utf-8')
print(json.dumps(summary,indent=2),flush=True)
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
