function validate_lift_approach60_matlab_20261106
% 独立升降小步标定候选的MATLAB前置模型；不调用现车或CAN。
% 只允许地址5、单次5000虚拟脉冲、30RPM、下行；STOP失应答保持待发不切VM。
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-06','lift-approach60');if ~exist(out,'dir'),mkdir(out);end
normal={'normal_scaled','normal_unscaled','negative_position','millis_wrap'};
faults={'bad_options','bad_position_sign','initial_not_stopped','enable_ack_lost','move_ack_lost', ...
 'target_mismatch','position_frozen','fractional_speed','driver_disabled','protection', ...
 'cancel','service_gap','read_failure','stop_ack_missing','stop_then_ack'};
results=struct('case',{},'result',{});checks=struct('name',{},'passed',{});
for k=1:numel(normal)
 r=simulate(normal{k});results(end+1)=struct('case',normal{k},'result',r); %#ok<AGROW>
 check([normal{k} ': exactly one finite move'],strcmp(r.state,'DONE') && r.moves==1);
 check([normal{k} ': no other address or home/disable/zero command'],r.only_node5 && ~r.forbidden_commands);
 check([normal{k} ': complete only after moved and speed raw zero'],r.raw_position_delta==r.expected_raw_delta && r.final_speed_raw==0);
end
for k=1:numel(faults)
 r=simulate(faults{k});results(end+1)=struct('case',faults{k},'result',r); %#ok<AGROW>
 check([faults{k} ': does not report ordinary done'],~strcmp(r.state,'DONE'));
 check([faults{k} ': no duplicate relative move'],r.moves<=1);
 check([faults{k} ': only node5'],r.only_node5 && ~r.forbidden_commands);
 check([faults{k} ': VM kept on after fault per existing user policy'],r.vm_requested);
 if strcmp(faults{k},'stop_ack_missing')
  check('STOP absence keeps pending and terminal unconfirmed',r.stop_pending && strcmp(r.state,'STOP_UNCONFIRMED') && ~r.stop_confirmed);
 elseif strcmp(faults{k},'stop_then_ack')
  check('STOP late acknowledgement is tracked without new move',r.stop_confirmed && ~r.stop_pending && r.stop_attempts>=3);
 else
  check([faults{k} ': STOP acknowledged'],r.stop_confirmed);
 end
end
% Use current measured10000p/6mm; sensitivity is declared, not measured bound.
initial_height=113;gain=[5/6,1,7/6];drop=52.8*gain;lowest=initial_height-drop;
check('nominal first approach is60.2mm',abs(initial_height-52.8-60.2)<1e-8);
check('declared endpoint uncertainty does not cross selected50mm',all(lowest>=50));
check('first approach stays below existing top-referenced105000p',10000+88000==98000 && 98000<105000);
check('direct63mm drop cannot claim same50mm lower bound',min(initial_height-63*gain)<50);
check('300RPM60RPMs nominal motion10.5sec',abs(88000*360/3200/(300*6)+300/60-10.5)<1e-8);
report=struct('engine','local MATLAB','version',version,'release',version('-release'), ...
 'candidate_step_pulses',88000,'candidate_rpm',300,'candidate_accel_rpm_s',60,'node',5, ...
 'timeout_ms',14000,'stop_confirm_timeout_ms',300,'maximum_service_gap_ms',300, ...
 'maximum_feedback_age_ms',400,'results',results,'checks',checks,'check_count',numel(checks), ...
 'failure_count',sum(~[checks.passed]),'model_checks_passed',all([checks.passed]), ...
 'plant_identified_from_current_lift',false,'real_lift_mm_scale_known',false, ...
 'new_measured_virtual_pulses_per_mm',10000/6,'declared_gain_factors',gain,'declared_drop_mm',drop, ...
 'lowest_model_gripper_bottom_mm',lowest,'declared_gain_is_measured_error_bound',false, ...
 'vm_auto_cut_on_fault',false,'command_restart_or_auto_repeat',false, ...
 'actual_hardware_operated',false,'new_firmware_deployed',false,'approach_height_nominal_mm',60.2,'direct_to50_released',false,'maximum_step_count',1,'motion_allowed',false);
write_json(fullfile(out,'matlab-result.json'),report);save(fullfile(out,'matlab-result.mat'),'report');
fprintf('MATLAB lift ruler: cases=%d checks=%d failures=%d, only node5 88000p300RPM, no auto VM cut\n',numel(results),report.check_count,report.failure_count);
assert(report.model_checks_passed,'Lift ruler model failed; do not implement');
 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed)); %#ok<AGROW>
 end
end

function r=simulate(name)
dt=.05;t=0;state='PREP';phase=0;move_start=NaN;stop_start=NaN;last_service=0;
scale=100;if strcmp(name,'normal_unscaled'),scale=10;end
pos0=123.4;if strcmp(name,'negative_position'),pos0=-900;end
delta=88000*360/3200;expected=round(delta*scale);moves=0;position=pos0;speed=0;
stop_pending=false;stop_confirmed=false;attempts=0;vm=true;stable=0;command_count=0;
trace=[];last_pos=0;initial_wait=0;
while t<=16
 if strcmp(name,'service_gap') && t>=.8 && t<1.2,t=t+dt;continue;end
 if stop_pending
  attempts=attempts+1;trace(end+1)=hex2dec('FE'); %#ok<AGROW>
  if ~strcmp(name,'stop_ack_missing') && (~strcmp(name,'stop_then_ack') || attempts>=3)
   stop_confirmed=true;stop_pending=false;state='STOPPED';break;
  end
  if t-stop_start>=.3,state='STOP_UNCONFIRMED';end
  t=t+dt;continue;
 end
 reason='';
 if t-last_service>.3+1e-9,reason='SERVICE_GAP';end
 if t>=14,reason='TIMEOUT';end
 if t>=.8
  if any(strcmp(name,{'cancel','stop_ack_missing','stop_then_ack'})),reason='CANCEL';end
  if strcmp(name,'driver_disabled'),reason='DISABLED';end
  if strcmp(name,'protection'),reason='PROTECTION';end
  if strcmp(name,'read_failure'),reason='COMMUNICATION';end
 end
 if ~isempty(reason),state='FAULT';stop_pending=true;stop_start=t;t=t+dt;continue;end
 last_service=t;
 if strcmp(state,'PREP')
  switch phase
   case 0
    trace(end+1)=hex2dec('1A'); %#ok<AGROW>
    if strcmp(name,'bad_options'),reason='OPTIONS';end
   case 1
    trace(end+1)=hex2dec('36'); %#ok<AGROW>
    if strcmp(name,'bad_position_sign'),reason='POSITION';end
   case 2
    trace(end+1)=hex2dec('35'); %#ok<AGROW>
    if strcmp(name,'initial_not_stopped')
     initial_wait=initial_wait+dt;
     if initial_wait>=2,reason='PREP_TIMEOUT';else,phase=1;end
    end
   case 3
    trace(end+1)=hex2dec('F3'); %#ok<AGROW>
    if strcmp(name,'enable_ack_lost'),reason='ENABLE';end
   case 4
    trace(end+1)=hex2dec('FD');moves=moves+1;move_start=t;command_count=command_count+1; %#ok<AGROW>
    if strcmp(name,'move_ack_lost'),reason='ACK_LOST';end
   otherwise
    trace(end+1)=hex2dec('33'); %#ok<AGROW>
    if strcmp(name,'target_mismatch'),reason='TARGET';else,state='MOVING';end
  end
  phase=phase+1;
 else
  % 声明的30RPM/60RPM/s对称加减速位置曲线。
  elapsed=t-move_start;acc=60*6;v=300*6;ta=v/acc;tc=delta/v-ta;total=2*ta+tc;
  if elapsed<ta,d=.5*acc*elapsed^2;speed=acc*elapsed;
  elseif elapsed<ta+tc,d=.5*acc*ta^2+v*(elapsed-ta);speed=v;
  elseif elapsed<total,remain=total-elapsed;d=delta-.5*acc*remain^2;speed=acc*remain;
  else,d=delta;speed=0;end
  position=pos0+d;
  if strcmp(name,'position_frozen'),position=pos0;end
  if strcmp(name,'fractional_speed') && elapsed>=total,speed=.5*6;end
  raw_delta=round((position-pos0)*scale);speed_raw=round(speed/6*10);last_pos=t;
  if t-last_pos>=.4,reason='POSITION_AGE';end
  if abs(raw_delta-expected)<=max(1,round(.3*scale)) && speed_raw==0 && elapsed>=total,stable=stable+1;else,stable=0;end
  if stable>=3,state='DONE';break;end
 end
 if ~isempty(reason),state='FAULT';stop_pending=true;stop_start=t;end
 t=t+dt;
end
r=struct('state',state,'moves',moves,'relative_move_transmissions',command_count, ...
 'only_node5',true,'forbidden_commands',any(ismember(trace,[hex2dec('FE00'),hex2dec('93'),hex2dec('9A')])), ...
 'raw_position_delta',round((position-pos0)*scale),'expected_raw_delta',expected, ...
 'final_speed_raw',round(speed/6*10),'stop_pending',stop_pending,'stop_confirmed',stop_confirmed, ...
 'stop_attempts',attempts,'vm_requested',vm,'time_s',t,'normal_time_model_only',true);
end
function write_json(path,value)
fid=fopen(path,'w','n','UTF-8');assert(fid~=-1);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(value));
end
