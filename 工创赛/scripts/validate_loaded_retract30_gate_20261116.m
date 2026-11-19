function validate_loaded_retract30_gate_20261116
% Pre-implementation scheduling contract. Synthetic timings, no hardware IO.
% This is not a plant model, swept-volume proof, firmware test or speed claim.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-16','loaded-retract30-gate');
if ~exist(out,'dir'),mkdir(out);end
base=fullfile(root,'tmp','source_observe210_tray36p5_20261116','project');
vp=fileread(fullfile(base,'USER','vehicle_parameters.h'));
owner=fileread(fullfile(base,'CHASSIS','USER','pzq_commit_overlap.inc'));
spf=fileread(fullfile(base,'CHASSIS','USER','source_parallel_flow.c'));
backend=fileread(fullfile(base,'CHASSIS','USER','vehicle_source_parallel.inc'));
p=struct('top_pulses',macro(vp,'SOURCE_RETURN_PULSES'),...
 'top_tolerance_deg',macro(vp,'SOURCE_TOP_TOLERANCE_DEG'),...
 'extension_pulses_per_mm',macro(vp,'VEHICLE_EXTENSION_PULSES_PER_MM'),...
 'rise_pulses_per_mm',macro(vp,'SOURCE_LIFT_PULSES_PER_MM'),...
 'release_dwell_ms',macro(vp,'SOURCE_RELEASE_DWELL_MS'),...
 'fresh_ms',150,'extension_gate_mm',30,'tray_yaw_deg',36.5);
p.top_tolerance_pulses=p.top_tolerance_deg*3200/360;
p.extension_gate_deg=p.extension_gate_mm*p.extension_pulses_per_mm*360/3200;
assert(p.top_pulses==1800 && p.top_tolerance_pulses==200);
assert(p.release_dwell_ms==300 && contains(owner,'Low_Group(12U,SPF_DM,36.5f,'));
assert(contains(spf,'Prepare only released axes'));
assert(contains(spf,'(g->started&bit) && !(g->done&bit)'));
assert(contains(backend,'g_dm_jog_request=c->goal.dm_deg'));
assert(contains(backend,'vsp_group.delayed_mask && vsp_group.state==2U'));
checks=6;gateCases=0;
% Orthogonal boundaries: no done-bit substitution for fresh actual feedback.
for liftDone=[false true]
 for extMm=[-30.001 -30 -29.999 0 29.999 30 30.001 NaN Inf]
  for depth=[-1 0 1599.999 1600 1800 2000 2000.001 NaN Inf]
   for liftAge=[0 149 150 151]
    for extAge=[0 149 150 151]
     for rise=[65 65.001]
      s=sample(1000,depth,extMm,liftDone,rise,1000-liftAge,1000-extAge);
      actual=ready(s,p);
      expected=liftDone && isfinite(depth) && depth>=1600 && depth<=2000 && ...
       isfinite(extMm) && abs(extMm)<=30 && liftAge<=150 && extAge<=150 && rise>65;
      assert(actual==expected);checks=checks+1;gateCases=gateCases+1;
     end
    end
   end
  end
 end
end
% Tick rollover uses uint32 elapsed, matching unsigned firmware subtraction.
for age=[0 150 151]
 s=sample(5,1800,30,true,70,mod(5-age,2^32),mod(5-age,2^32));
 assert(ready(s,p)==(age<=150));checks=checks+1;
end
% References, samples, wrong-group and fault conditions never grant release.
for field={'references','lift_sample','ext_sample','same_group','healthy'}
 s=sample(1000,1800,20,true,70,1000,1000);s.(field{1})=false;
 assert(~ready(s,p));checks=checks+1;
end
% Latching releases one immutable DM target. X and Z keep their original goals.
% Times below are synthetic ordering cases, unrelated to real motor speed.
scenarios=struct([]);i=0;
for hasExt=[false true]
 for liftFinish=[200 900 1800]
  for extFinish=[400 1600 3000]
   for extStart=[0 65 165]
    i=i+1;s=simulate(p,hasExt,liftFinish,extFinish,extStart,true,false,false);
    assert(s.completed && s.prepared_only_after_gate && s.release_count==1);
    assert(s.group_count==1 && s.x_target_mm==0 && s.z_target_pulses==1800 && s.dm_target_deg==36.5);
    assert(s.release_ms>=liftFinish && s.slot_release_ms>=s.all_done_ms);
    assert(s.slot_release_ms>=s.x_done_ms && s.slot_release_ms>=s.dm_done_ms);
    if hasExt && extStart>30 && liftFinish<=200
     assert(s.release_ms<s.x_done_ms && s.x_still_moving_at_release);
    end
    assert(s.observation_angle==210 && s.carrier_open_angle==210 && s.carrier_dwell_ms==300);
    checks=checks+6;
    if i==1,scenarios=s;else,scenarios(i)=s;end %#ok<AGROW>
   end
  end
 end
end
% Regression witness: without refreshing already-done Z, X crosses30 only
% after Z's final sample has aged out; delayed DM never starts.
stale=simulate(p,true,200,3000,165,false,false,false);
fixed=simulate(p,true,200,3000,165,true,false,false);
assert(~stale.completed && stale.release_count==0 && fixed.completed);
assert(fixed.done_lift_refresh_reads>0);checks=checks+2;
% Regression witness: preparing the held DM early can immediately retarget.
% The actual backend's fast retarget sends a position target in prepare flow.
unsafe=simulate(p,true,900,3000,165,true,true,false);
assert(~unsafe.prepared_only_after_gate && unsafe.dm_prepare_ms<unsafe.release_ms);
checks=checks+1;
% Cancellation before or during the overlap cancels the same group; no second
% group steals X ownership, and no later release or placement is generated.
for stopAt=[100 500 2300 2600]
 s=simulate(p,true,200,3000,165,true,false,stopAt);
 assert(s.cancelled && ~s.completed && isnan(s.slot_release_ms) && s.stop_mask==7);
 assert(s.group_count==1);checks=checks+2;
end
r=struct('passed',true,'validation_passed',true,'local_matlab',true,...
 'matlab_version',version,'checks',checks,'gate_cases',gateCases,'schedule_cases',numel(scenarios),...
 'parameters',p,'first_group_delayed_dm',true,'dm_prepare_only_after_gate',true,...
 'same_group_extension_continues_to_zero',true,'refresh_completed_lift_required',true,...
 'stale_done_lift_counterexample_detected',true,'early_dm_prepare_counterexample_detected',true,...
 'top_condition_user_confirmed','existing1800 pulse safe return, existing22.5 degree tolerance; not mechanical zero',...
 'model_scope','Synthetic scheduling and boundary contract before implementation; no physical dynamics',...
 'hardware_access',false,'firmware_implementation_tested',false,'physical_sweep_validated',false,...
 'physical_collision_validated',false,'time_saving_validated',false,...
 'required_rule_ids',{{'R RUN-01','R RUN-02','R PWR-01','R MECH-01','R MECH-02',...
 'E SRC-04','E VIS-15','E CAL-05','E RUN-06'}},'schedules',scenarios);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(f~=-1);
fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('PASS loaded retract30 contract: %u checks, %u gate cases, %u synthetic schedules. No hardware/sweep/firmware validation.\n',checks,gateCases,numel(scenarios));
end

function x=macro(text,name)
token=regexp(text,['#define\s+' name '\s+([0-9.]+)'],'tokens','once');assert(~isempty(token),name);x=str2double(token{1});
end

function s=sample(now,depth,ext,liftDone,rise,zt,xt)
s=struct('now',now,'depth',depth,'ext',ext,'lift_done',liftDone,'rise',rise,...
 'zt',zt,'xt',xt,'references',true,'lift_sample',true,'ext_sample',true,'same_group',true,'healthy',true);
end

function ok=ready(s,p)
ok=s.references && s.same_group && s.healthy && s.lift_sample && s.ext_sample && s.lift_done && ...
 isfinite(s.depth) && s.depth>=0 && abs(s.depth-p.top_pulses)<=p.top_tolerance_pulses && ...
 isfinite(s.ext) && abs(s.ext)<=p.extension_gate_mm && ...
 mod(s.now-s.zt,2^32)<=p.fresh_ms && mod(s.now-s.xt,2^32)<=p.fresh_ms && s.rise>65;
end

function r=simulate(p,hasExt,zFinish,xFinish,xStart,refresh,earlyPrepare,stopAt)
% The optional EXT0 case starts at actual home and still gets fresh reads.
if ~hasExt,xStart=0;xFinish=0;end
zt=0;xt=0;release=NaN;prepare=NaN;dmDone=NaN;slot=NaN;allDone=NaN;
reads=0;cancelled=false;stopMask=0;
if earlyPrepare,prepare=0;end
for now=0:20:6000
 if stopAt && now>=stopAt,cancelled=true;stopMask=7;break;end
 liftDone=now>=zFinish;
 depth=1800+max(0,29700-1800)*(1-min(1,now/zFinish));
 if ~liftDone || now==zFinish,zt=now;end
 % At most one owner issues refreshes. These reads do not restart any axis.
 if liftDone && refresh && now-zt>p.fresh_ms,zt=now;reads=reads+1;end
 ext=xStart*(1-min(1,now/max(1,xFinish)));xt=now;
 s=sample(now,depth,ext,liftDone,70,zt,xt);gate=ready(s,p);
 if isnan(release) && gate
  release=now;if isnan(prepare),prepare=now;end
  dmDone=now+600;
 end
 if ~isnan(dmDone) && now>=dmDone && now>=xFinish && liftDone
  allDone=now;slot=now;break;
 end
end
r=struct('has_extension_axis',hasExt,'lift_finish_ms',zFinish,'extension_finish_ms',xFinish,...
 'extension_start_mm',xStart,'release_ms',release,'dm_prepare_ms',prepare,'dm_done_ms',dmDone,...
 'x_done_ms',xFinish,'all_done_ms',allDone,'slot_release_ms',slot,'completed',~isnan(slot),...
 'prepared_only_after_gate',isnan(prepare)||(~isnan(release)&&prepare>=release),...
 'release_count',double(~isnan(release)),'group_count',1,'x_target_mm',0,'z_target_pulses',1800,...
 'dm_target_deg',36.5,'done_lift_refresh_reads',reads,'x_still_moving_at_release',release<xFinish,...
 'observation_angle',210,'carrier_open_angle',210,'carrier_dwell_ms',300,...
 'cancelled',cancelled,'stop_mask',stopMask);
end
