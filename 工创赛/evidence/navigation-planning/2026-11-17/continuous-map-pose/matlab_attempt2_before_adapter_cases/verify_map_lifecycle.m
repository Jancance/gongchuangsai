function verify_map_lifecycle
% State/lifecycle model, not a measurement replay or actual C execution.
% World: x right, y up, heading CCW from +x; initial front points +y.
out=fileparts(mfilename('fullpath'));checks=0;
for start=1:2
 s=begin([],1,start,100);assert(s.state==1&&~s.valid&&s.run==1);checks=checks+1;
 % Old cache must not establish a new round baseline.
 s=step(s,120,false,[90 90 90 90],110,[0 0],0,false);assert(s.state==1);checks=checks+1;
 % Complete post-begin cache and yaw while idle establish the only origin.
 s=step(s,140,false,[130 130 130 130],130,[0 0],0,false);assert(s.valid&&s.state==2);checks=checks+1;
 origin=s.xy;offset=s.offset;
 % Local SetPose/ResetRecords/stage/session changes do not enter this model.
 for plan=1:6
  t=140+plan*20;
  s=step(s,t,true,[t t t t],t,[20 0],0,false);
  assert(norm(s.xy-origin-[0 plan*20])<1e-8&&s.offset==offset);checks=checks+1;
 end
 s=step(s,280,true,[280 280 280 280],280,[0 0],90,false);
 s=step(s,300,true,[300 300 300 300],300,[50 0],90,false);
 assert(norm(s.xy-origin-[-50 120])<1e-8);checks=checks+1;
 % Duplicate sample is not another displacement.
 before=s.xy;s=step(s,310,false,[300 300 300 300],300,[50 0],90,false);
 assert(isequal(s.xy,before)&&s.sample==300&&s.age==10);checks=checks+1;
 % Fresh source sample, not publish time; missing one source eventually gaps.
 s=step(s,700,false,[700 700 700 300],700,[0 0],90,false);
 assert(s.state==3&&~s.valid&&s.sample==300&&s.age==400);checks=checks+1;
 frozen=s.xy;s=step(s,720,false,[720 720 720 720],720,[99 99],0,false);
 assert(s.state==3&&~s.valid&&isequal(s.xy,frozen));checks=checks+1;
 same=begin(s,1,start,740);old=begin(s,0,start,740);
 assert(isequal(same,s)&&isequal(old,s));checks=checks+2;
 % Explicit genuinely new round can begin; not a same-run gap recovery.
 s=begin(s,2,start,800);assert(s.state==1&&s.run==2&&~s.valid);checks=checks+1;
 s=step(s,810,true,[810 810 810 810],810,[0 0],0,false);
 assert(s.state==3&&~s.valid);checks=checks+1;
 % A middle-stage test has no established field origin.
 s=begin(s,3,0,900);assert(s.state==4&&~s.valid);checks=checks+1;
 % Source arbitration chooses exactly one newest raw sample per wheel.
 [ms,raw]=select(100,1,110,2);assert(ms==110&&raw==2);checks=checks+1;
 [ms,raw]=select(120,3,110,2);assert(ms==120&&raw==3);checks=checks+1;
 [ms,raw]=select(120,3,120,3);assert(ms==120&&raw==3);checks=checks+1;
end
% Independent age checks for yaw and wheels, and fault freeze.
s=begin([],1,1,100);s=step(s,120,false,[120 120 120 120],120,[0 0],0,false);
s=step(s,220,false,[220 220 220 220],120,[0 0],0,false);assert(~s.valid&&s.state==3);checks=checks+1;
s=begin(s,2,2,300);s=step(s,320,false,[320 320 320 320],320,[0 0],0,false);
s=step(s,330,false,[330 330 330 330],330,[10 0],0,true);assert(~s.valid&&s.state==3);checks=checks+1;
% Actual caller order: Mission state1 waits while CAN gathers asynchronous
% idle diagnostics; all four mask bits alone do NOT imply interpolation-ready.
for already_holding=[false true]
 first=inf(1,4);latest=-inf(1,4);begun=0;released=false;ready_at=NaN;
 for transaction=1:24
  now=transaction*10;
  if mod(transaction,3)==0
   wheel=mod(transaction/3-1,4)+1;
   first(wheel)=min(first(wheel),now);latest(wheel)=now;
  end
  common=min(latest);can_interpolate=all(first<=common)&all(latest>=common);
  % yaw history starts at begin and refreshes every CAN cycle in this model.
  ready=can_interpolate&&all(latest>=begun)&&all(now-latest<400)&&common>=begun;
  if transaction==12,assert(all(isfinite(latest))&&~ready);checks=checks+1;end
  if ready&&~released,ready_at=now;released=true;end
  % existing startup state1 is kept; no motion has been submitted yet.
  assert(~released||ready);checks=checks+1;
 end
 assert(released&&ready_at==210);checks=checks+1;
end
% Intermediate-stage tests have no map start and do not wait on map tracking.
for stage=2:8
 map_required=(stage==1);assert(~map_required);checks=checks+1;
end
r=struct('status','PASS_LIFECYCLE_MODEL_ONLY','checks',checks,'hardware_accessed',false,'actual_c_executed',false,'source','RAW_WHEEL_HWT','navigation_calibration_applied',false,'start_heading_deg',90,'wheel_max_age_ms',400,'yaw_max_age_ms',100,'limitations','Synthetic complete-frame lifecycle model. Actual interpolated raw wheel/HWT adapter must be tested separately; no physical placement, polling latency or accuracy claim.');
fid=fopen(fullfile(out,'matlab_summary.json'),'w');fprintf(fid,'%s\n',jsonencode(r));fclose(fid);fprintf('PASS %d lifecycle assertions\n',checks);
end
function s=begin(s,run,start,now)
if ~isempty(s)&&run<=s.run,return;end
s=struct('run',run,'start',start,'begin',now,'state',1,'valid',false,'xy',[0 0],'offset',90,'sample',0,'age',inf);
if start==0,s.state=4;else,s.xy=[2250 150];if start==2,s.xy(1)=150;end,end
end
function s=step(s,now,active,wms,yms,delta,yaw,fault)
s.valid=false;s.age=now-s.sample;
if s.state>=3,return;end
if s.state==1
 if active,s.state=3;return;end
 if any(wms<s.begin)||yms<s.begin||any(now-wms>=400)||now-yms>=100,return;end
 s.state=2;s.sample=min([wms yms]);s.age=now-s.sample;s.valid=true;return;
end
if fault||any(now-wms>=400)||now-yms>=100,s.state=3;return;end
common=min([wms yms]);if common>s.sample
 angle=(s.offset+yaw)*pi/180;s.xy=s.xy+[cos(angle)*delta(1)-sin(angle)*delta(2) sin(angle)*delta(1)+cos(angle)*delta(2)];s.sample=common;
end
s.age=now-s.sample;s.valid=true;
end
function [ms,raw]=select(bms,braw,dms,draw)
ms=bms;raw=braw;if dms>bms,ms=dms;raw=draw;end
end
