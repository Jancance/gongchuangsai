"""X42接口/规划/共同负载模型对比；不复现厂商闭源固件，不连接硬件。"""
from pathlib import Path
import numpy as np
import json,csv,sys,math,itertools,hashlib
R=Path(__file__).resolve().parents[1];E=R/'evidence/p1-foundation/2026-11-01/x42-emm-x-comparison';O=R/'output/x42-emm-x-comparison-20261101';E.mkdir(parents=True,exist_ok=True);O.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,str(R/'tmp/dm_sim_libs'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
# 可准确计算的接口区别。当前实际细分未读，逐个候选而不是假定16。
position_modes={'Emm16':1.8/16,'Emm64':1.8/64,'Emm256':1.8/256,'X_default':.1,'X_0p01':.01}
targets=np.linspace(0,360,200003);resolution=[]
for name,lsb in position_modes.items():
 error=np.round(targets/lsb)*lsb-targets
 resolution.append(dict(mode=name,step_deg=lsb,command_rmse_deg=float(np.sqrt(np.mean(error**2))),command_max_error_deg=float(np.max(np.abs(error))),scope='command quantization, NOT actual positioning accuracy'))
# matched acceleration profiles; Emm integer speed or optional0.1RPM vs X0.1RPM.
speed_steps={'Emm_default':1.,'Emm_scale0p1':.1,'X':.1};speed_t=np.arange(0,4.0001,.002);ideal_speed=np.interp(speed_t,[0,.8,1.8,2.6,4],[0,5.35,5.35,-5.35,-5.35]);speed_metrics=[];speed_traces={}
for name,lsb in speed_steps.items():
 cmd=np.round(ideal_speed/lsb)*lsb;speed_traces[name]=cmd
 speed_metrics.append(dict(mode=name,step_rpm=lsb,command_rmse_rpm=float(np.sqrt(np.mean((cmd-ideal_speed)**2))),scope='command quantization only'))
assert np.array_equal(speed_traces['Emm_scale0p1'],speed_traces['X'])
# Emm档位的精确公式与X独立RPM/s进行比较，不把档位当RPM/s。
accels=np.arange(1,256);emm_accel=1/((256-accels)*50e-6);planning=[]
for requested in [50,80,100,250,500,1000,2500,5000,10000,20000,30000,45055,65535]:
 i=int(np.argmin(abs(emm_accel-requested)));actual=emm_accel[i]
 planning.append(dict(requested_rpm_s=requested,emm_nearest_acc=int(accels[i]),emm_rpm_s=float(actual),emm_relative_error_pct=float(abs(actual-requested)/requested*100),x_integer_rpm_s=requested,scope='known planner input formula, not motor acceleration measurement'))
assert abs(emm_accel[235]-1000)<1e-8 and abs(max(emm_accel)-20000)<1e-8
# 共同二阶负载代理模型：两模式用完全相同控制器，仅隔离输入量化影响。
# 厂商FOC参数未知，绝不把自行给X整定更优参数的结果当作实际固件胜负。
plants=[dict(inertia=j,static_friction=fs,load=load,zeta=zeta,bandwidth_hz=hz,encoder_bits=bits) for j,fs,load,zeta,hz,bits in itertools.product([.0002,.002,.008],[0.,.03],[0.,.02],[.7,1.2],[8.,16.],[14,16])]
M=len(position_modes);P=len(plants);N=M*P
j=np.tile([p['inertia'] for p in plants],M);fs=np.tile([p['static_friction'] for p in plants],M);load=np.tile([p['load'] for p in plants],M);zeta=np.tile([p['zeta'] for p in plants],M);wn=2*np.pi*np.tile([p['bandwidth_hz'] for p in plants],M);encoder_step=2*np.pi/2**np.tile([p['encoder_bits'] for p in plants],M)
q_goal_deg=90.037;goal=np.repeat([np.round(q_goal_deg/lsb)*lsb*np.pi/180 for lsb in position_modes.values()],P);q=np.zeros(N);w=np.zeros(N);tau=np.zeros(N);max_tau=np.zeros(N)
dt=.0001;end=2.;vmax=100.*2*np.pi/60;acc=1000.*2*np.pi/60
# Emmacc236与X1000RPM/s恰好相等，排除加速参数不同的混淆。
ta=np.minimum(vmax/acc,np.sqrt(goal/acc));vp=acc*ta;tc=np.maximum(0,(goal-acc*ta*ta)/vp);total=2*ta+tc
records=[];nominal_plant=plants.index(dict(inertia=.002,static_friction=.03,load=.02,zeta=.7,bandwidth_hz=8.,encoder_bits=16));sel=np.arange(M)*P+nominal_plant
for k in range(round(end/dt)+1):
 t=k*dt
 u=np.clip(t-ta-tc,0,ta);ref=np.where(t<ta,.5*acc*t*t,np.where(t<ta+tc,.5*acc*ta*ta+vp*(t-ta),.5*acc*ta*ta+vp*tc+vp*u-.5*acc*u*u));vr=np.where(t<ta,acc*t,np.where(t<ta+tc,vp,np.maximum(0,vp-acc*u)));ref=np.where(t>=total,goal,ref);vr=np.where(t>=total,0,vr)
 enc=np.round(q/encoder_step)*encoder_step;target_tau=np.clip(j*wn**2*(ref-enc)+2*zeta*j*wn*(vr-w),-.8,.8)
 tau+=(target_tau-tau)*dt/.001;net=tau-load-.01*w;stationary=(abs(w)<1e-4)&(abs(net)<=fs);direction=np.where(abs(w)<1e-4,np.sign(net),np.sign(w));new_w=w+(net-.75*fs*direction)/j*dt;cross=(w*new_w<0)&(abs(net)<=fs);new_w=np.where(stationary|cross,0,new_w);q+=.5*(w+new_w)*dt;w=new_w;max_tau=np.maximum(max_tau,abs(tau))
 if k%10==0:records.append(np.column_stack([np.full(M,t),q[sel]*180/np.pi,w[sel]*60/(2*np.pi),ref[sel]*180/np.pi]))
rows=[]
for i,(name,lsb) in enumerate(position_modes.items()):
 for c,plant in enumerate(plants):
  n=i*P+c;rows.append(dict(mode=name,plant=c,**plant,command_target_deg=goal[n]*180/np.pi,model_final_error_deg=q[n]*180/np.pi-q_goal_deg,model_max_torque_nm=max_tau[n],scope='same surrogate controller across firmware labels; not real Emm/X firmware response'))
assert np.all(np.isfinite(q)) and max(max_tau)<=.80000001
traces=np.asarray(records)
for i,name in enumerate(position_modes):np.savetxt(E/(name+'-proxy-trace.csv'),traces[:,i,:],delimiter=',',header='time_s,proxy_position_deg,proxy_speed_rpm,planner_reference_deg',comments='')
for name,data in [('position_resolution.csv',resolution),('speed_resolution.csv',speed_metrics),('acceleration_mapping.csv',planning),('common_plant_sensitivity.csv',rows)]:
 with (E/name).open('w',newline='',encoding='utf-8-sig') as f:
  writer=csv.DictWriter(f,fieldnames=data[0].keys());writer.writeheader();writer.writerows(data)
assumptions={'scope':'Offline interface/planner comparison plus equal-controller plant sensitivity only. Manufacturer firmware internals are NOT reproduced.','actual_hardware_record':'X42S_V2.0; FW_Emm; four chassis nodes1..4, CAN500k, S_Vel_IS Disable user report2026-10-17. Not new hardware readback.','actual_microsteps':'not read back;16/64/256 are alternatives, not asserted current values','plants':plants,'models':list(position_modes),'model_count':N,'time_step_s':dt,'motion_deg':q_goal_deg,'speed_cap_rpm':100,'matched_acceleration_rpm_s':1000,'emm_acc236_matches_x1000':True,'proxy_control':'Identical second-order error feedback torque across all labels; bandwidth8/16Hz, damping0.7/1.2, current lag1ms and torque cap0.8Nm ASSUMED. No actual motor PID readback.','encoder_assumptions':'14 and16 bits swept; V2.0 preference for16bit is not proof of physical installed encoder bits. Do not use0.1deg X CAN feedback as internal encoder quantization.','limitations':['No internal manufacturerFOC reproduction or unbiased empirical ranking of firmware controllers','No wheel slip, Mecanum kinematics, HWT/odometry/chassis path or vehicle accuracy acceptance','Torque/friction/inertia are unmeasured; no actual load or current calibration','No actual protocol switch, flash, motion, auto-calibration or motor configuration writes','Existing P1 prepare-control F3 E2 remains unresolved; simulation does not fix or clear it'], 'decision':'No actual-precision superiority established. Retain Emm; X is eligible for later actual same-condition comparison, not automatically switched on surrogate results.'}
(E/'assumptions.json').write_text(json.dumps(assumptions,indent=2),encoding='utf-8')
result={'position_resolution':resolution,'speed_resolution':speed_metrics,'acceleration_mapping':planning,'proxy_case_count':N,'proxy_plant_count':P,'decision':assumptions['decision']};(E/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
fig,axs=plt.subplots(2,2,figsize=(12,8));names=list(position_modes)
axs[0,0].bar(np.arange(M),[z['command_rmse_deg'] for z in resolution],color=['#64748b','#64748b','#64748b','#d97706','#059669']);axs[0,0].set_xticks(np.arange(M),names,rotation=18);axs[0,0].set(title='Command quantization RMS (not positioning accuracy)',ylabel='deg')
for name,cmd in speed_traces.items():
 mask=(speed_t<1.2);axs[0,1].step(speed_t[mask],cmd[mask],where='post',label=name,lw=1.2)
axs[0,1].plot(speed_t[speed_t<1.2],ideal_speed[speed_t<1.2],'k:',label='ideal request');axs[0,1].set(title='Low-speed command: scaled Emm and X overlap',xlabel='Time (s)',ylabel='RPM');axs[0,1].legend(fontsize=8)
for i,name in enumerate(names):axs[1,0].plot(traces[:,i,0],(traces[:,i,1]-q_goal_deg),label=name,lw=1)
axs[1,0].set(xlim=(.55,2),ylim=(-1,.3),title='Same proxy plant/control, terminal error zoom',xlabel='Time (s)',ylabel='deg');axs[1,0].legend(fontsize=8)
rr=[z['requested_rpm_s'] for z in planning];ee=[z['emm_rpm_s'] for z in planning];axs[1,1].loglog(rr,ee,'o-',label='nearest Emm nonzero acc');axs[1,1].loglog(rr,rr,'s--',label='X requested integer acc');axs[1,1].set(title='Acceleration command coverage',xlabel='Requested RPM/s',ylabel='Representable RPM/s');axs[1,1].legend(fontsize=8)
for ax in axs.flat:ax.grid(alpha=.2)
fig.suptitle('X42S Emm vs X: known interface facts + UNIDENTIFIED shared proxy model',fontsize=13);fig.text(.5,.012,'No real firmware dynamics or chassis acceptance. Finer command units do not guarantee better physical accuracy.',ha='center',fontsize=9);fig.tight_layout(rect=[0,.03,1,.96]);fig.savefig(O/'comparison.png',dpi=150);plt.close(fig)
# Standalone concise preview for chat.
fig,axs=plt.subplots(1,2,figsize=(11,3.6));axs[0].bar(np.arange(M),[z['command_rmse_deg'] for z in resolution]);axs[0].set_xticks(np.arange(M),names,rotation=18);axs[0].set(title='Position command RMS quantization',ylabel='deg; NOT actual accuracy')
for name,cmd in speed_traces.items():
 mask=speed_t<1.2;axs[1].step(speed_t[mask],cmd[mask],where='post',label=name,lw=1.2)
axs[1].set(title='Commanded RPM: X = Emm with0.1 scaling',xlabel='Time(s)',ylabel='RPM');axs[1].legend(fontsize=8)
for ax in axs:ax.grid(alpha=.2)
fig.suptitle('Offline X42 interface comparison - no physical firmware winner established');fig.tight_layout();fig.savefig(O/'preview.png',dpi=150);plt.close(fig)
print(json.dumps({'proxy_case_count':N,'resolution':resolution,'speed_metrics':speed_metrics,'decision':assumptions['decision']},indent=2),flush=True)
