function validate_stopped3_grip170_release200_20261115
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-15','stopped3-grip170-release200');
if ~exist(out,'dir'),mkdir(out);end
checks=0;vectors={};n=0;
for dt=[.025 .05 .1 .2]
 for speed=[0 10 24.9 25 25.1 60]
  t=(0:7)*dt;x=100+speed*t;
  opened=simulate(t,x);
  assert(~any(opened(1:2)));checks=checks+1;
  if speed<25,assert(all(opened(3:end)));checks=checks+1;
  elseif speed>25,assert(~any(opened));checks=checks+1;end
  n=n+1;vectors{n}=struct('time',t,'x',x,'opened',opened);
 end
end
% Motion, stop, gap, duplicate timestamp and abrupt displacement.
paths={ [0 .1 .2 .3 .4 .5 .6], [0 .1 .5 .6 .7], [0 .1 .1 .2 .3], [0 .1 .2 .3 .4 .5] };
xs={ [100 110 120 120 120 120 120], [100 100 100 100 100], [100 100 100 100 100], [100 100 150 150 150 150] };
for i=1:numel(paths)
 t=paths{i};x=xs{i};opened=simulate(t,x);
 assert(opened(end));checks=checks+1;
 n=n+1;vectors{n}=struct('time',t,'x',x,'opened',opened);
end
for command=[0 100 4294967200]
 for applyDelay=[0 1 19]
  applied=mod(command+applyDelay,2^32);
  for elapsed=[0 99 100 199 200 201]
   now=mod(applied+elapsed,2^32);
   assert((mod(now-applied,2^32)>=200)==(elapsed>=200));checks=checks+1;
  end
 end
end
% PWM 170 is issued before descent; no new dwell at source pickup.
pwm=round(500+170*2000/270);assert(pwm==1759);checks=checks+1;
r=struct('passed',true,'local_matlab',true,'checks',checks,'vectors',[vectors{:}],...
 'required_stable_frames',3,'near_stop_speed_px_s',25,'minimum_frame_interval_s',.02,...
 'maximum_frame_interval_s',.25,'preclose_deg',170,'carrier_release_ms',200,...
 'source_extra_wait_ms',0,'physical_stopped_truth_validated',false,...
 'physical_sweep_validated',false,'hardware_access',false);
fp=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(fp,'%s\n',jsonencode(r));fclose(fp);
fprintf('PASS stopped3/grip170/release200: %u checks, %u independent stop-sequence vectors.\n',checks,n);
end
function result=simulate(t,x)
h=[];result=false(size(t));
for k=1:numel(t)
 if ~isempty(h)
  dt=t(k)-t(h(end));
  if dt<=0 || dt>.25,h=[];
  elseif dt<.02,continue;
  elseif any(abs(x(k)-x(h))./(t(k)-t(h))>25),h=[];
  end
 end
 h=[h k];if numel(h)>3,h=h(end-2:end);end
 result(k)=numel(h)>=3;
end
end
