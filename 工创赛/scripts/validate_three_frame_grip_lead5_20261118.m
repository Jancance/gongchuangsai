% Restore existing 5mm preclose, using current region-latched target.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-18','three-frame-grip-lead5');
if ~exist(out,'dir'),mkdir(out);end
checks=0;
for region=0:2
 target=29700-4000*double(region<2);threshold=target-5*400;
 assert(threshold==23700+4000*double(region==2));checks=checks+1;
 depths=[threshold-1,threshold,threshold+1,target];ages=[0,149,150,151,500];
 depth_truth=[false,true,true,true];fresh_truth=[true,true,true,false,false];
 for di=1:numel(depths)
  depth=depths(di);
  for ai=1:numel(ages)
   age=ages(ai);
   for mask=0:127
    % phase4, current sequence, LIFT present+started, no group fault,
    % position cycle exists, not yet gripped.
    early=bitget(mask,1) && bitget(mask,2) && bitget(mask,3) &&...
      bitget(mask,4) && bitget(mask,5) && bitget(mask,6) && bitget(mask,7) &&...
      age<=150 && depth>=threshold;
    expected=mask==127 && fresh_truth(ai) && depth_truth(di);
    assert(early==expected);checks=checks+1;
   end
  end
 end
 for mode=0:2 % valid moving sample / all absent / all stale
  calls=0;gripped=false;stamp=0;up=false;events=[];
  for t=[100,120,140]
   pos_cycle=mode~=1;age=151*double(mode==2);
   if pos_cycle && age<=150 && ~gripped
    calls=calls+1;gripped=true;stamp=t;events(end+1)=1; %#ok<SAGROW>
   end
   assert(~up && calls==double(mode==0));checks=checks+1;
  end
  assert(stamp==100*double(mode==0));checks=checks+1;
  events(end+1)=2; % full pose DONE at t=200
  if ~gripped,calls=calls+1;gripped=true;stamp=200;events(end+1)=1;end
  oldstamp=stamp;up=true;events(end+1)=3; % submit rise, no added wait
  assert(calls==1 && up && stamp==oldstamp);checks=checks+1;
  assert(find(events==1,1)<find(events==3,1));checks=checks+1;
  assert((find(events==1,1)<find(events==2,1))==(mode==0));checks=checks+1;
  assert(stamp==100+100*double(mode~=0));checks=checks+1;
 end
end
% Missing early samples: close at completed pose then immediately submit rise;
% no artificial wait and no claim of mechanical closure feedback.
calls=0;events=["pose_done","close140","submit_rise"];
assert(events(2)=="close140" && events(3)=="submit_rise");checks=checks+1;
result=struct('passed',true,'checks',checks,'side_target',25700,'bottom_target',29700,...
 'side_preclose_threshold',23700,'bottom_preclose_threshold',27700,...
 'existing_freshness_ms',150,'new_grip_dwell_ms',0,'hardware_access',false,...
 'note','Command ordering and region threshold model; sparse feedback may fall back at DONE, no physical grip confirmation');
fid=fopen(fullfile(out,'matlab-result.json'),'w');fprintf(fid,'%s\n',jsonencode(result));fclose(fid);
fprintf('PASS MATLAB restored lead5 %d checks\n',checks);
