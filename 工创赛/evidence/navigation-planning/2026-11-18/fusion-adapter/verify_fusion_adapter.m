function verify_fusion_adapter
out=fileparts(mfilename('fullpath'));checks=0;vectors=[];cfg=struct('min_dt',20,'max_dt',100,'max_age',100,'stop_v',2,'stop_w',.5);
for dt=[20 50 100]
 for direction=[0 37 90 180 -90]
  for speed=[0 20 200 400]
   for corr=[0 100]
    a=map(1000,[300 300],170);b=a;b.ms=1000+dt;b.now=b.ms;b.updates=2;b.raw=a.raw+speed*dt/1000*[cosd(direction) sind(direction)];b.xy=b.raw+[corr -corr];b.h=wrap(a.h+dt*.1);
    [state,o,r]=observe(cfg,struct(),a);assert(r==0&&~o.valid);checks=checks+1;
    [state,o,r]=observe(cfg,state,b);assert(r==1&&o.valid);checks=checks+1;
    assert(norm(o.v-speed*[cosd(direction) sind(direction)])<1e-8&&abs(o.omega-100)<1e-8&&isequal(o.xy,b.xy)&&~o.stopped);checks=checks+1;
    vectors=[vectors;dt direction speed corr b.h o.v o.omega b.xy];
    old=o;b.now=b.now+1;b.flags=4;b.epoch=99;[same,o,r]=observe(cfg,state,b);old.now=b.now;assert(r==1&&isequal(o,old)&&isequal(state,same));checks=checks+1;
   end
  end
 end
end
% min_dt retains anchor, no speed0; enough small fresh samples eventually complete window.
a=map(1000,[300 300],90);[s,o,r]=observe(cfg,struct(),a);
for dt=[1 5 10 19]
 b=a;b.ms=1000+dt;b.now=b.ms;b.updates=dt+1;b.raw=a.raw+[dt*.4 0];b.xy=b.raw;[s,o,r]=observe(cfg,s,b);assert(r==0&&~o.valid&&s.anchor.ms==1000);checks=checks+1;
end
b.ms=1020;b.now=1020;b.updates=21;b.raw=a.raw+[8 0];b.xy=b.raw;[s,o,r]=observe(cfg,s,b);assert(r==1&&abs(o.v(1)-400)<1e-8);checks=checks+1;
% Duplicate/first/zero raw delta are estimated stationary only, never automatic stop.
a=map(1000,[300 300],90);[s,o,r]=observe(cfg,struct(),a);assert(~o.stopped);checks=checks+1;
b=a;b.ms=1020;b.now=1020;b.updates=2;[s,o,r]=observe(cfg,s,b);assert(r==1&&~o.stopped&&o.stationary);checks=checks+1;
b.stop=true;b.stop_ms=1020;b.stop_run=1;[same,o,r]=observe(cfg,s,b);assert(r==1&&~o.stopped&&isequal(same,s));checks=checks+1;
b.ms=1040;b.now=1040;b.updates=3;[s,o,r]=observe(cfg,s,b);assert(r==1&&o.stopped);checks=checks+1;
old=s;b.now=1141;[s,o,r]=observe(cfg,s,b);assert(r==2&&~o.valid&&isempty(fieldnames(s)));checks=checks+1;
% Fresh source resumes with two samples. Stale/GAP never bridges estimation history.
a=map(1200,[350 300],90);[s,o,r]=observe(cfg,s,a);assert(r==0);checks=checks+1;b=a;b.ms=1220;b.now=1220;b.updates=2;[s,o,r]=observe(cfg,s,b);assert(r==1);checks=checks+1;
for kind=1:8
 bad=b;
 if kind==1,bad.valid=false;elseif kind==2,bad.gap=true;elseif kind==3,bad.local=true;elseif kind==4,bad.now=bad.ms+101;elseif kind==5,bad.raw(1)=NaN;elseif kind==6,bad.begin=bad.ms+1;elseif kind==7,bad.ms=bad.now+1;else,bad.id=2;end
 [next,o,r]=observe(cfg,s,bad);assert(r==2&&~o.valid);checks=checks+1;
end
% Same source core mutation rejects without consuming previous baseline/cache.
b.now=b.ms;bad=b;bad.raw(1)=bad.raw(1)+1;[same,o,r]=observe(cfg,s,bad);assert(r==2&&isequal(same,s));checks=checks+1;
% Map epoch restart or update rollback reprimes, not cross-origin velocity.
b.ms=1240;b.now=1240;b.begin=1230;b.raw=[800 900];b.xy=b.raw;[s,o,r]=observe(cfg,s,b);assert(r==0&&s.anchor.ms==1240);checks=checks+1;
b.ms=1260;b.now=1260;b.updates=1;[s,o,r]=observe(cfg,s,b);assert(r==0);checks=checks+1;
% Excess gap reprimes. Old timestamps never rewind valid history.
b.ms=1400;b.now=1400;b.updates=2;[s,o,r]=observe(cfg,s,b);assert(r==0&&s.anchor.ms==1400);checks=checks+1;
bad=b;bad.ms=1390;[same,o,r]=observe(cfg,s,bad);assert(r==2&&isequal(same,s));checks=checks+1;
% Wrapped map heading (already includes its world offset), no 358deg jump.
a=map(1000,[300 300],179);b=a;b.ms=1020;b.now=1020;b.h=-179;b.updates=2;[s,~,~]=observe(cfg,struct(),a);[s,o,r]=observe(cfg,s,b);assert(r==1&&abs(o.omega-100)<1e-8&&o.h==-179);checks=checks+1;
% stop bad/future/old-run/old-before-begin never creates physical stopping evidence.
for kind=1:5
 a=map(1000,[300 300],90);[s,~,~]=observe(cfg,struct(),a);b=a;b.ms=1020;b.now=1020;b.updates=2;b.stop=true;b.stop_ms=1020;b.stop_run=1;
 if kind==1,b.stop=false;elseif kind==2,b.stop_run=2;elseif kind==3,b.stop_ms=1021;elseif kind==4,b.stop_ms=800;else,b.stop_ms=899;end
 [s,o,r]=observe(cfg,s,b);assert(r==1&&~o.stopped);checks=checks+1;
end
% Quantized positions: 1ms count steps exceed the unchanged 2mm/s reference margin.
quantum=pi*77/65536;noise=[];
for dt=[1 5 10 20 50 100]
 errs=[];
 for phase=linspace(0,quantum,101)
  x0=phase;x1=phase+400*dt/1000;v=(round(x1/quantum)-round(x0/quantum))*quantum*1000/dt;errs=[errs v-400];
 end
 noise=[noise;dt min(errs) max(errs)];assert(max(abs(errs))<=quantum*1000/dt+1e-8);checks=checks+1;
end
assert(quantum*1000>2&&quantum*1000/20<2);checks=checks+1;
% Average estimate lag under deceleration: not a current-state braking certificate.
for dt=[20 50 100]
 mean_v=400+200*dt/2000;assert(mean_v>400&&abs(mean_v-400-200*dt/2000)<1e-10);checks=checks+1;
end
writematrix(vectors,fullfile(out,'adapter_vectors.csv'));writematrix(noise,fullfile(out,'quantization_scan.csv'));
summary=struct('status','PASS','checks',checks,'vectors',size(vectors,1),'scope','Copied map raw finite-window ESTIMATED feedback only; no instantaneous velocity, physical stop, live caller or deployment proof','nominal_min_dt_ms',20,'max_dt_ms',100,'quantum_mm',quantum,'model_only',true,'hardware_accessed',false);
f=fopen(fullfile(out,'matlab_summary.json'),'w');fprintf(f,'%s',jsonencode(summary));fclose(f);disp(summary);
end
function h=wrap(h),h=mod(h+180,360)-180;end
function a=map(ms,xy,h)
a=struct('ms',ms,'now',ms,'begin',900,'updates',1,'id',1,'raw',xy,'xy',xy,'h',h,'valid',true,'gap',false,'local',false,'flags',0,'epoch',0,'stop',false,'stop_ms',0,'stop_run',0);
end
function [s,o,status]=observe(c,s,a)
o=struct('valid',false,'ms',0,'now',0,'xy',[0 0],'h',0,'v',[0 0],'omega',0,'stopped',false,'stationary',false);status=2;
if a.id~=1,return;end
if ~all(isfinite([a.raw a.xy a.h])) || ~a.valid || a.gap || a.local || a.ms>a.now || a.ms<a.begin || a.now-a.ms>c.max_age
 s=struct();return;
end
core=struct('ms',a.ms,'begin',a.begin,'raw',a.raw,'xy',a.xy,'h',a.h,'id',a.id);
stop=a.stop&&a.stop_run==1&&a.stop_ms<=a.now&&a.stop_ms>=a.begin&&a.now-a.stop_ms<=c.max_age;
if isempty(fieldnames(s)) || a.begin~=s.latest.begin || a.updates<s.updates
 s=struct('anchor',a,'latest',core,'updates',a.updates,'ready',false,'cached',o,'stop_ms',0);status=0;return;
end
if a.ms<s.latest.ms,return;end
if a.ms==s.latest.ms
 if ~isequal(core,s.latest),return;end
 status=0;
 if s.ready
  if s.cached.stopped && (~stop || a.now-s.stop_ms>c.max_age),return;end
  o=s.cached;o.now=a.now;status=1;
 end
 return;
end
dt=a.ms-s.anchor.ms;
if dt>c.max_dt,s=struct('anchor',a,'latest',core,'updates',a.updates,'ready',false,'cached',o,'stop_ms',0);status=0;return;end
s.latest=core;s.updates=a.updates;s.ready=false;status=0;
if dt<c.min_dt,return;end
v=(a.raw-s.anchor.raw)*1000/dt;omega=wrap(a.h-s.anchor.h)*1000/dt;
if norm(v)>10000||abs(omega)>3600,s=struct();status=2;return;end
o=struct('valid',true,'ms',a.ms,'now',a.now,'xy',a.xy,'h',a.h,'v',v,'omega',omega,'stopped',stop,'stationary',norm(v)<=c.stop_v&&abs(omega)<=c.stop_w);
s.anchor=a;s.ready=true;s.cached=o;s.stop_ms=a.stop_ms;status=1;
end
