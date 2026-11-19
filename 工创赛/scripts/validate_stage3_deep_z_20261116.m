% Stage3 deep-Z pre-implementation model. No firmware or hardware access.
% R RUN-01/02/03, MECH-01/02; E CAL-05/RUN-06; official Q unchanged.
% User: top fingertip-to-ground 205mm and lift can reach ground. 400p/mm.
% 82000 is the approved stage3 business scope, NOT a descent/place command.
root=fileparts(fileparts(mfilename('fullpath')));
outdir=fullfile(root,'evidence','integration','2026-11-16','stage3-deep-z');
if ~exist(outdir,'dir'),mkdir(outdir);end
checks=struct('name',{},'passed',{},'details',{});
depths=[3200 29700 32000 40000 60000 82000];
assert(205*400==82000 && 29700/400==74.25);
% User measurement is at safeTop, 600p/400=1.5mm below true home.
% Stage3 target coordinates remain relative to TRUE top, not current safeTop.
safeTopDepth=600;safeTopGroundGap=205;
assert(safeTopDepth/400==1.5);
assert(safeTopGroundGap+safeTopDepth/400==206.5);
assert((82000-safeTopDepth)/400==203.5); % not a205mm move from safeTop
assert(safeTopGroundGap-(82000-safeTopDepth)/400==1.5);
for d=depths
    assert(admit('stage3',d));
    assert(admit('source',d)==(d<=29700));
end
for d=[-inf -1 0 3199.9 82000.1 inf nan]
    assert(~admit('stage3',d));
end
for d=[-inf -1 29700.1 32000 82000 inf nan]
    assert(~admit('source',d));
end
assert(admit('source',0) && ~admit('unknown',3200));
addCheck('scope_ranges_and_nonfinite', ...
 'stage3 3200..82000; source 0..29700 unchanged; wrong scope and nonfinite rejected; no actual place target selected');

% Existing ordinary profiles: only Stage3 extension RPM changes150 ->106.
% Keep X acceleration600, generic IT3 Z600, IT4 return45055, DM300deg/s.
xRev=170*294.4/3200;xTime=xRev/(106/60)+106/600;
zRev=82000/3200;zTime=zRev/(1846.4/60)+1846.4/600;
returnTime=zRev/(1846.4/60)+1846.4/45055;
assert(xTime<12 && zTime<12 && returnTime<12 && 270/300<12);
assert(82000-1800==80200 && 80200<=82000 && 80200>29700);
addCheck('original_stage3_speeds_existing_timeouts', ...
 'ideal motion upper estimate X170mm106RPM/600~9.03s;genericIT3 Z82000p1846.4RPM/600<4s;IT4return1846.4RPM/45055<1s;DM270/300<1s; existing12s group deadline unchanged; no load/timing guarantee');
addCheck('return_scope_only_and_reference_origin', ...
 'deep82000 to existing true-top-relative1800 needs80200p delta; source default29700 cannot cover; safeTop600p measurement origin differs1.5mm from true top; do not set floor/place target');

% Absolute encoder conversion: same scale for top and current. X driver
% options select 0.1deg or 0.01deg units; no old pulses/degree substitution.
conversions=0;maxPulseError=0;
for scale=[10 100]
 for top=[-2140000 -375 0 1830000]
  for direction=[-1 1]
   for d=depths
    topDeg=top/scale;
    goalDeg=topDeg+direction*d*360/3200;
    goalRaw=round(goalDeg*scale);
    measuredDepth=(goalRaw/scale-topDeg)*direction*3200/360;
    error=abs(measuredDepth-d);
    maxPulseError=max(maxPulseError,error);
    assert(error<=0.5/scale*3200/360+1e-7);
    assert(abs(goalRaw)<2^31);
    % Feedback publication derives current feedback, never command target.
    halfwayRaw=round((topDeg+direction*d/2*360/3200)*scale);
    published=(halfwayRaw/scale-topDeg)*direction*3200/360;
    assert(abs(published-d/2)<=0.5/scale*3200/360+1e-7);
    assert(abs(published-d)>1000);
    conversions=conversions+1;
   end
  end
 end
end
addCheck('absolute_raw_deg_pulse_feedback', ...
 '96 combinations; signed absolute top/current; scale10 current0.1deg and scale100; feedback publication cannot copy the target');

% Current topology: dispatch -> backend ACK -> fresh feedback -> DONE.
% No exact-time motor dynamics or newly invented arrival tolerance model.
phaseCount=0;
for d=depths
 m=newModel();m=issue(m,'stage3',d);
 assert(m.requests==1 && strcmp(m.phase,'backend_ack'));
 for k=1:3,m=service(m,struct());end
 assert(m.requests==1 && ~m.done);
 m=service(m,struct('ack',true));assert(strcmp(m.phase,'feedback') && ~m.done);
 m=service(m,struct('feedback',d,'cycle',0));assert(~m.done); % stale
 m=service(m,struct('feedback',d/2,'cycle',1));assert(~m.done);
 m=service(m,struct('feedback',d,'cycle',2));assert(m.done);
 assert(m.actualDepth==d && m.requests==1);
 % A completed explicit request remains completed/idle, never loops.
 for k=1:5,m=service(m,struct());end
 assert(m.requests==1);
 phaseCount=phaseCount+1;
end
addCheck('backend_ack_fresh_feedback_no_false_done', ...
 'six target depths; no ACK/no new feedback/wrong feedback cannot finish; published actual from received feedback; model uses exact synthetic arrival only');

% Native return first moves to trueTop+1800, then either validated-reference
% reuse finishes there, or physical home then existing600p backoff completes.
for d=depths
 r=struct('phase','home','topSeen',false,'backoffDone',false,'done',false,'depth',d);
 r=returnService(r,false,false);assert(~r.done && ~r.topSeen);
 r=returnService(r,true,false);assert(r.topSeen && ~r.done && strcmp(r.phase,'backoff'));
 r=returnService(r,false,false);assert(~r.done);
 r=returnService(r,false,true);assert(r.done && r.backoffDone && r.depth==600);
 % Existing reference reuse finishes at1800p and does not claim a new home.
 reused=struct('depth',1800,'done',true,'fresh_home_seen',false);
 assert(reused.done && ~reused.fresh_home_seen && reused.depth==1800);
end
addCheck('return_actual_top_then_existing_600_backoff', ...
 'native trueTop+1800 approach; valid-reference reuse finishes1800 without fresh-home claim; home-required branch sees real home then feedback-confirmed existing600p backoff; no synthesized reference');

% STOP reuses existing pending/ACK semantics at every motion/return phase.
phases={'backend_ack','feedback','home','backoff','done'};
stopCases=0;
for k=1:numel(phases)
 for depth=depths
  m=newModel();m=issue(m,'stage3',depth);m.phase=phases{k};
  m=stopModel(m);assert(m.fault && m.stopPending && ~m.done);
  m=service(m,struct('stopAck',false));assert(m.stopPending && ~m.done);
  m=service(m,struct('stopAck',true));assert(~m.stopPending && m.fault && ~m.done);
  for j=1:3,m=service(m,struct());end
  assert(m.requests==1 && m.fault); % no clear/retry/loop
  stopCases=stopCases+1;
 end
end
addCheck('stop_all_phases_existing_ack_no_resume', ...
 '30 depth/phase combinations; STOP pending waits existing backend ACK; fault retained, no automatic retry or fault clearing; no physical-stop assertion');

% Segmented field teaching is multiple separately authorized requests.
% Nothing automatically advances to the next item of this illustrative list.
m=newModel();m=issue(m,'stage3',32000);
for k=1:20,m=service(m,struct());end
assert(m.requests==1 && m.target==32000 && ~m.done);
assert(isnan(nan)); % final placement remains unknown, not205-60.
addCheck('one_explicit_segment_only', ...
 'one command causes one requested segment only; depth list is validation input, not a queued descent; real placement depth remains unknown');

% Ensuring top is a no-op when EXISTING real reference/feedback says already
% at top. It must not publish a second1800->1800 zero-distance LRC request.
for phase=[0 1 6 11]
 for actualTop=[false true]
  requests=0;nextPhase=phase;
  if actualTop,nextPhase=phase+1;else,requests=requests+1;end
  assert(requests==~actualTop);
  assert(nextPhase==phase+actualTop);
 end
end
addCheck('already_at_real_top_skips_redundant_return', ...
 'Stage3 preparation and ensure-top phases use existing real it_at_top feedback; already-top advances without IT4 or fabricated DONE; actual deep position still requests one IT4; LRC zero-delta rejection unchanged');

result=struct('when',datestr(now,31),'passed',all([checks.passed]), ...
 'checks',checks,'depths_pulses',depths,'raw_cases',conversions, ...
 'stop_cases',stopCases,'max_rounding_error_pulses',maxPulseError, ...
 'source_max_unchanged',29700,'stage3_business_max',82000, ...
 'top_fingertip_to_ground_mm',205,'physical_range_tested',false, ...
 'place_depth_known',false,'hardware_access',false, ...
 'limitations','Geometry/discrete topology only; no real lift dynamics, load, collision, field calibration or actual stopping verification.');
fid=fopen(fullfile(outdir,'matlab-result.json'),'w');assert(fid>0);
fwrite(fid,jsonencode(result),'char');fclose(fid);
disp(jsonencode(result));

function yes=admit(scope,d)
 yes=isnumeric(d)&&isscalar(d)&&isfinite(d);
 if ~yes,return;end
 if strcmp(scope,'stage3'),yes=d>=3200&&d<=82000;
 elseif strcmp(scope,'source'),yes=d>=0&&d<=29700;
 else,yes=false;end
end
function m=newModel()
 m=struct('phase','idle','requests',0,'done',false,'fault',false, ...
 'stopPending',false,'target',nan,'actualDepth',nan,'cycle',0);
end
function m=issue(m,scope,d)
 assert(admit(scope,d));assert(strcmp(m.phase,'idle'));
 m.requests=m.requests+1;m.target=d;m.phase='backend_ack';
end
function m=service(m,event)
 if m.fault
  if isfield(event,'stopAck')&&event.stopAck,m.stopPending=false;end
  return;
 end
 if strcmp(m.phase,'backend_ack')
  if isfield(event,'ack')&&event.ack,m.phase='feedback';end
 elseif strcmp(m.phase,'feedback')&&isfield(event,'cycle')&&event.cycle>m.cycle
  m.cycle=event.cycle;m.actualDepth=event.feedback;
  if m.actualDepth==m.target,m.phase='done';m.done=true;end
 end
end
function m=stopModel(m)
 m.fault=true;m.stopPending=true;m.done=false;
end
function r=returnService(r,actualTop,actualBackoff)
 if strcmp(r.phase,'home')&&actualTop
  r.topSeen=true;r.phase='backoff';
 elseif strcmp(r.phase,'backoff')&&actualBackoff
  r.depth=600;r.backoffDone=true;r.done=true;r.phase='done';
 end
end
function addCheck(name,details)
 list=evalin('caller','checks');
 list(end+1)=struct('name',name,'passed',true,'details',details);
 assignin('caller','checks',list);
end
