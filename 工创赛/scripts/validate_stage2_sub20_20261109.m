function validate_stage2_sub20_20261109
% End of timer = third confirmed release/route request, not route arrival.
% Timing uses the one verified run; overlaps and target duration are scenarios.
root=fileparts(fileparts(mfilename('fullpath')));
ev=fullfile(root,'evidence','integration','2026-11-07','stage2-mainline');
r=jsondecode(fileread(fullfile(ev,'three-overlap-live-20261109.json')));
assert(r.complete && abs(r.three_to_release_s-34.843)<.001);
old=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-06', ...
    'source-vision-new-pose','dwell3-budget','matlab-result.json')));
assert(old.minimum_upward_lift_to_equal_other_material_top_mm==60 && ~old.motion_allowed);

names={'down','left','right'};
fields={'aim','down','extend','grip','rise','retract','to_slot','release'};
m=struct();
for f=1:numel(fields)
    value=zeros(1,3);
    for k=1:3, value(k)=step(r,[names{k},'_',fields{f}]); end
    m.(fields{f})=value;
end
zero=[0,0,0]; serial=sum(m.aim+m.down+m.extend+m.grip+m.rise+m.retract+m.to_slot+m.release);
assert(serial<r.three_to_release_s);
bench=struct('serial_steps_s',serial,'host_and_inter_step_s',r.three_to_release_s-serial);

% Reuse the observed 0.39 s from the former 160 -> 140 bench experiment only
% as an indicative budget. No shorter close is enabled on the current hardware.
fast=m; fast.grip=[.39 .39 .39];
[requested,parts_requested]=sequence(fast,zero,zero,fast.rise,fast.rise,.35);
% Geometric historical case: +60 mm before retract if another piece is crossed.
% Estimate offset from linear partial travel, without treating it as evidence.
travel_mm=118800/1600;
old_clear_s=60/travel_mm*fast.rise;
[historical,parts_hist]=sequence(fast,zero,[.2 .2 .2],old_clear_s,old_clear_s,.35);
% Lower bound for unchanged observed stages: remove grip and open time,
% allow immediate retract and incoming lowering, reserve no software latency.
% This shows scheduling sensitivity, not a physical or competition guarantee.
[ideal_parallel,parts_ideal]=sequence(fast,[.75 .75 .75],[.25 .25 .25],zero,zero,0);

% Target budget for a shorter *verified* lift envelope and faster retraction.
% These are required future measured stage budgets; not setpoints.
target=fast; target.down=[1.55 1.55 1.55];target.rise=[1.55 1.55 1.55];
target.retract=[0 1.20 1.25];target.release=[.43 .43 .43];
[goal,parts_goal]=sequence(target,[.8 .35 .35],[.24 .24 .24], ...
    [.75 .28 .28],[.75 .75 .75],.35);
[slower,~]=sequence(scale(target,1.05),[.8 .35 .35],[.24 .24 .24], ...
    [.75 .28 .28]*1.05,[.75 .75 .75]*1.05,.35);
assert(requested>20 && historical>20 && goal<20 && slower>goal);

% Validate the release -> Route3 + DM0 contract independently of the timeline.
% A verified 0-degree sweep and an already safe lift/extension pose are required.
checks=0;
for placed=0:3
 for mask=0:7
  for fresh=0:1
   for route=0:1
    for sweep=0:1
     for safe=0:1
      for stow=[0 180]
       [body,dm]=departure(placed,mask,fresh,route,sweep,safe,stow);
       expect=(placed==3 && mask==7 && fresh && route && sweep && safe && stow==0);
       assert(body==expect && dm==expect);
       checks=checks+1;
      end
     end
    end
   end
  end
 end
end

out=struct('passed',true,'measured_release_s',r.three_to_release_s, ...
 'measured_transport_s',r.three_to_transport_s,'baseline',bench, ...
 'measured_command_durations_s',m,'requested_overlaps_no_loaded_early_retract_s',requested, ...
 'old_60_mm_clearance_model_s',historical,'old_60_mm_clearance_not_verified',true, ...
 'optimistic_measured_stage_parallel_s',ideal_parallel,'target_budget_s',goal, ...
 'target_5pct_slower_s',slower,'requested_parts',parts_requested, ...
 'historical_parts',parts_hist,'ideal_parts',parts_ideal,'target_parts',parts_goal, ...
 'target_stage_budgets_s',target,'departure_model_cases',checks, ...
 'hardware_enabled',false,'current_parallel_geometry_verified',false);
fid=fopen(fullfile(ev,'stage2-sub20-matlab-20261109.json'),'w','n','UTF-8');
assert(fid>0);fprintf(fid,'%s\n',jsonencode(out));fclose(fid);
fprintf('Measured %.3fs; raw serial %.3fs; serial host gaps %.3fs.\n',r.three_to_release_s,serial,bench.host_and_inter_step_s);
fprintf('Descent+extension and quick close only: %.3fs; 60mm historical clearance scenario: %.3fs.\n',requested,historical);
fprintf('Optimistic measured-stage overlaps: %.3fs. Target schedule %.3fs (5%% slower %.3fs).\n',ideal_parallel,goal,slower);
fprintf('PASS: %d departure admission cases; no hardware geometry claimed.\n',checks);
end

function x=step(r,label)
if iscell(r.steps), rows=r.steps;else,rows=num2cell(r.steps);end
j=find(cellfun(@(s)strcmp(s.label,label),rows));
if isempty(j),x=0;return;end
assert(isscalar(j));x=rows{j}.elapsed_s;
end

function [t,parts]=sequence(m,aimLead,gripLead,retractClear,turnClear,dispatch)
% Incoming yaw can overlap only the pre-cleared high portion of descent.
% Extension starts at the taught yaw; 140 closing cannot begin before X settles.
z0=max(0,m.aim-aimLead);
zdone=z0+m.down;
xdone=m.aim+m.extend;
clamp_start=max(xdone,zdone-gripLead);
approach=max(zdone,clamp_start+m.grip);
retracted=zeros(1,3);
side=m.retract>0;
retracted(side)=retractClear(side)+m.retract(side);
% Never turn loaded until its sweep height AND actual retracted zero are ready.
turn_start=max(turnClear,retracted);
carry=max(m.rise,turn_start+m.to_slot);
t=sum(approach+carry+m.release)+dispatch;
parts=struct('approach_s',approach,'carry_s',carry,'release_s',m.release,'dispatch_s',dispatch);
end

function m=scale(m,k)
f=fieldnames(m);
for i=1:numel(f),m.(f{i})=m.(f{i})*k;end
end

function [body,dm]=departure(placed,mask,fresh,route,sweep,safe,stow)
body=placed==3 && mask==7 && fresh && route && sweep && safe && stow==0;
dm=body;
end
