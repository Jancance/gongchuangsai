function verify_fusion_adapter_v9
% Pure copied-value adapter contract, before product implementation.
out=fileparts(mfilename('fullpath'));checks=0;vectors=[];
p=plan();groups=struct('M1',0,'M2',0,'M3',0,'M4',0,'M5',0,'M6',0,'M7',0);mark=0;
for dt=[20 50 100]
 for direction=[0 37 90 180 -90]
  for speed=[0 20 200 400]
   for corr=[0 100]
    a=map(1000,[300 300],170);b=advance(a,dt);
    b.raw=a.raw+speed*dt/1000*[cosd(direction) sind(direction)];b.xy=b.raw+[corr -corr];b.corr=b.xy-b.raw;b.h=wrap(a.h+dt*.1);
    [s,o,r]=observe(p,struct(),a);assert(r.status==0&&~o.valid);checks=checks+1;
    [s,o,r]=observe(p,s,b);assert(r.status==1&&o.valid);checks=checks+1;
    assert(norm(o.v-speed*[cosd(direction) sind(direction)])<1e-8&&abs(o.omega-100)<1e-8&&isequal(o.xy,b.xy)&&~o.stopped&&o.source==1);checks=checks+1;
    vectors=[vectors;dt direction speed corr a.h b.h o.v o.omega b.xy]; %#ok<AGROW>
    old=o;b.now=b.now+1;b.age=b.age+1;b.flags=4;b.epoch=99;b.scope=3;b.source=2;b.nav=1;b.wheel(:)=b.ms+1;b.angle=b.ms+1;b.published=b.now;
    [same,o,r]=observe(p,s,b);old.now=b.now;assert(r.status==1&&isequal(o,old)&&isequal(s,same)&&r.flags==4&&r.epoch==99);checks=checks+1;
   end
  end
 end
end
groups.M7=groups.M7+checks-mark;mark=checks;
% All six caller identity fields and all three actual map identity fields.
for field=1:6
 a=map(1000,[300 300],90);[s,~,~]=observe(p,struct(),a);b=advance(a,20);b.identity(field)=b.identity(field)+1;
 [same,o,r]=observe(p,s,b);assert(r.status==2&&~o.valid&&isequal(same,s));checks=checks+1;
end
for field=1:3
 a=map(1000,[300 300],90);[s,~,~]=observe(p,struct(),a);b=advance(a,20);b.map_identity(field)=b.map_identity(field)+1;
 [same,o,r]=observe(p,s,b);assert(r.status==2&&~o.valid&&isequal(same,s));checks=checks+1;
end
% A legitimate new plan signature/binding starts an independent window.
a=map(1000,[300 300],90);[s,~,~]=observe(p,struct(),a);b=advance(a,20);[s,~,~]=observe(p,s,b);
q=p;q.identity(5)=2;q.signature=2;b.identity=q.identity;
[s,o,r]=observe(q,s,b);assert(r.status==0&&~o.valid&&s.anchor.ms==b.ms&&isequal(s.identity,q.identity));checks=checks+1;
q=p;q.signature=3;b=map(1100,[300 300],90);[s,~,r]=observe(q,s,b);assert(r.status==0&&s.signature==3);checks=checks+1;
groups.M1=groups.M1+checks-mark;mark=checks;
% Short fresh intervals retain the valid anchor, and invalidate old cache.
a=map(1000,[300 300],90);[s,~,~]=observe(p,struct(),a);
for dt=[1 5 10 19]
 b=advance(a,dt);b.raw=a.raw+[dt*.4 0];b.xy=b.raw;
 [s,o,r]=observe(p,s,b);assert(r.status==0&&~o.valid&&s.anchor.ms==1000);checks=checks+1;
end
b=advance(a,20);b.raw=a.raw+[8 0];b.xy=b.raw;[s,o,r]=observe(p,s,b);assert(r.status==1&&o.v(1)==400);checks=checks+1;
b=advance(b,1);[s,o,r]=observe(p,s,b);assert(r.status==0&&~o.valid&&~s.ready&&s.anchor.ms==1020);checks=checks+1;
groups.M5=groups.M5+checks-mark;mark=checks;
% First, duplicate, zero raw delta and software-only completion cannot stop.
a=map(1000,[300 300],90);[s,o,r]=observe(p,struct(),a);assert(~o.stopped);checks=checks+1;
b=advance(a,20);[s,o,r]=observe(p,s,b);assert(r.status==1&&~o.stopped&&r.stationary);checks=checks+1;
b.stop=true;b.stop_ms=b.ms;b.stop_run=1;[same,o,r]=observe(p,s,b);assert(r.status==1&&~o.stopped&&r.stop_pending&&isequal(same,s));checks=checks+1;
b=advance(b,20);[s,o,r]=observe(p,s,b);assert(r.status==1&&o.stopped);checks=checks+1;
% Revocation invalidates the whole result, never rewrites cached stopped to 0.
b.stop=false;[same,o,r]=observe(p,s,b);assert(r.status==2&&~o.valid&&r.stop_pending&&isequal(same,s));checks=checks+1;
% A cached physical-stop assertion cannot be retained beyond its own age.
b.stop=true;b.now=b.stop_ms+101;b.angle=b.now;b.wheel(:)=b.now;
[same,o,r]=observe(p,s,b);assert(r.status==2&&~o.valid&&isequal(same,s));checks=checks+1;
groups.M6=groups.M6+checks-mark;mark=checks;
% Fresh healthy recovery needs two samples; no old history crosses a gap.
for kind=1:13
 a=map(1000,[300 300],90);b=advance(a,20);[s,~,~]=observe(p,struct(),a);[s,~,~]=observe(p,s,b);bad=b;
 switch kind
 case 1,bad.valid=0;case 2,bad.state=3;case 3,bad.based=0;case 4,bad.continuity=1;case 5,bad.reasons=1;
 case 6,bad.local=1;case 7,bad.now=bad.ms+101;case 8,bad.raw(1)=NaN;case 9,bad.xy(2)=Inf;
 case 10,bad.h=NaN;case 11,bad.corr(1)=Inf;case 12,bad.angle=u32(bad.now-100);case 13,bad.wheel(3)=u32(bad.now-400);
 end
 [s,o,r]=observe(p,s,bad);assert(r.status==2&&~o.valid&&isempty(fieldnames(s)));checks=checks+1;
 c=map(1200,[350 300],90);[s,o,r]=observe(p,s,c);assert(r.status==0&&~o.valid);checks=checks+1;
 c=advance(c,20);[s,o,r]=observe(p,s,c);assert(r.status==1&&o.valid);checks=checks+1;
end
groups.M3=groups.M3+checks-mark;mark=checks;
% Old pose timestamp preserves history; invalid source times clear health.
a=map(1000,[300 300],90);b=advance(a,20);[s,~,~]=observe(p,struct(),a);[s,~,~]=observe(p,s,b);
for kind=1:5
 bad=b;
 switch kind
 case 1,bad.ms=1019;case 2,bad.ms=1021;case 3,bad.begin=1021;case 4,bad.angle=1021;case 5,bad.wheel(2)=1021;
 end
 [same,o,r]=observe(p,s,bad);assert(r.status==2&&~o.valid);if kind==1,assert(isequal(same,s));else,assert(isempty(fieldnames(same)));end;checks=checks+1;
end
groups.M2=groups.M2+checks-mark;mark=checks;
% Same timestamp conflicting core is rejected, without consuming an anchor.
for kind=1:5
 bad=b;
 switch kind
 case 1,bad.raw(1)=bad.raw(1)+1;case 2,bad.raw(2)=bad.raw(2)+1;case 3,bad.xy(1)=bad.xy(1)+1;case 4,bad.xy(2)=bad.xy(2)+1;
 case 5,bad.h=bad.h+1;
 end
 [same,o,r]=observe(p,s,bad);assert(r.status==2&&~o.valid&&isequal(same,s));checks=checks+1;
end
bad=b;bad.corr=[20 30];[same,o,r]=observe(p,s,bad);assert(r.status==1&&isequal(same,s)&&isequal(r.corr,bad.corr));checks=checks+1;
groups.M4=groups.M4+checks-mark;mark=checks;
% begin epoch reset, update rollback and excess dt prime a fresh baseline.
c=advance(b,20);c.begin=1030;c.raw=[800 900];c.xy=c.raw;[s,o,r]=observe(p,s,c);assert(r.status==0&&~o.valid&&s.anchor.ms==1040);checks=checks+1;
c=advance(c,20);c.updates=1;[s,o,r]=observe(p,s,c);assert(r.status==0&&s.anchor.ms==1060);checks=checks+1;
c=advance(c,101);[s,o,r]=observe(p,s,c);assert(r.status==0&&s.anchor.ms==1161);checks=checks+1;
% Nondecreasing counter is metadata; only rollback triggers a new baseline.
c=advance(c,20);c.updates=s.updates;[s,o,r]=observe(p,s,c);assert(r.status==1&&o.valid);checks=checks+1;
bad=c;bad.ms=u32(c.ms-1);bad.updates=0;[same,o,r]=observe(p,s,bad);assert(r.status==2&&~o.valid&&isequal(same,s));checks=checks+1;
groups.M4=groups.M4+checks-mark;mark=checks;
% uint32 rollover and half-cycle boundary use unsigned delta explicitly.
a=map(4294967280,[300 300],179);a.begin=4294967200;b=advance(a,20);b.h=-179;b.raw=[308 300];b.xy=b.raw;
[s,~,~]=observe(p,struct(),a);[s,o,r]=observe(p,s,b);assert(r.status==1&&o.ms==4&&abs(o.omega-100)<1e-8&&o.v(1)==400);checks=checks+1;
b.now=5;[same,o,r]=observe(p,s,b);assert(r.status==1&&o.now==5&&isequal(same,s));checks=checks+1;
for offset_ms=[2147483648 4294967295]
 bad=b;bad.ms=u32(b.ms+offset_ms);[same,o,r]=observe(p,s,bad);assert(r.status==2&&~o.valid);checks=checks+1;
end
groups.M2=groups.M2+checks-mark;mark=checks;
% Map heading already has world offset: only wrapped map-heading difference.
for offset=[0 90 -37]
 a=map(1000,[300 300],wrap(179+offset));b=advance(a,20);b.h=wrap(-179+offset);
 [s,~,~]=observe(p,struct(),a);[s,o,r]=observe(p,s,b);assert(r.status==1&&abs(o.omega-100)<1e-8&&o.h==b.h);checks=checks+1;
end
groups.M7=groups.M7+checks-mark;mark=checks;
% Stop evidence is independently time/run checked, not a validity prerequisite.
for kind=1:7
 a=map(1000,[300 300],90);b=advance(a,20);b.stop=1;b.stop_ms=1020;b.stop_run=1;
 switch kind
 case 1,b.stop=0;case 2,b.stop_run=2;case 3,b.stop_ms=1021;case 4,b.stop_ms=918;case 5,b.stop_ms=899;case 6,b.stop=2;case 7,b.stop_ms=u32(b.now+2147483648);
 end
 [s,~,~]=observe(p,struct(),a);[s,o,r]=observe(p,s,b);assert(r.status==1&&~o.stopped);checks=checks+1;
end
a=map(1000,[300 300],90);b=advance(a,20);b.stop=1;b.stop_ms=1020;b.stop_run=1;b.raw=[308 300];b.xy=b.raw;
[s,~,~]=observe(p,struct(),a);[s,o,r]=observe(p,s,b);assert(r.status==1&&o.stopped&&~r.stationary&&o.v(1)==400);checks=checks+1;
groups.M6=groups.M6+checks-mark;mark=checks;
% Model plan/domain checks (real Prepare validation is reused by product caller).
for kind=1:11
 q=p;
 switch kind
 case 1,q.valid=0;case 2,q.signature=0;case 3,q.count=1;case 4,q.count=33;case 5,q.box_count=33;case 6,q.max_age=0;
 case 7,q.max_age=19;case 8,q.max_age=1001;case 9,q.stop_v=NaN;case 10,q.stop_w=Inf;case 11,q.model_only=0;
 end
 [s,o,r]=observe(q,struct(),map(1000,[300 300],90));assert(r.status==2&&~o.valid&&isempty(fieldnames(s)));checks=checks+1;
end
groups.M1=groups.M1+checks-mark;mark=checks;
% Fixed 20..100ms interval: max_age is not silently used to shrink it.
q=p;q.max_age=99;[s,o,r]=observe(q,struct(),map(1000,[300 300],90));assert(r.status==2&&~o.valid);checks=checks+1;
groups.M1=groups.M1+checks-mark;mark=checks;
% Preserve current map audit, including unsupported/boundary/tail and four scopes.
for direction=0:4
 for flags=0:7
  a=map(1000,[300 300],90);b=advance(a,20);b.direction=direction;b.flags=flags;b.scope=3;b.nav=1;b.source=2;b.corr=[1 -2];b.xy=b.raw+b.corr;
  [s,~,~]=observe(p,struct(),a);[s,o,r]=observe(p,s,b);
  assert(r.status==1&&isequal(o.xy,b.xy)&&r.direction==direction&&r.flags==flags&&isequal(r.corr,b.corr)&&r.scope==3&&r.source==2&&~r.mixed_verified);checks=checks+1;
 end
end
groups.M7=groups.M7+checks-mark;mark=checks;
% Count quantum / window sweep: unchanged reference margins are not widened.
quantum=pi*77/65536;noise=[];
for dt=[1 5 10 20 50 100]
 errs=[];
 for phase=linspace(0,quantum,101)
  x0=phase;x1=phase+400*dt/1000;v=(round(x1/quantum)-round(x0/quantum))*quantum*1000/dt;errs=[errs v-400]; %#ok<AGROW>
 end
 noise=[noise;dt min(errs) max(errs)];assert(max(abs(errs))<=quantum*1000/dt+1e-8);checks=checks+1; %#ok<AGROW>
end
assert(quantum*1000>2&&quantum*1000/20<2);checks=checks+1;
for dt=[20 50 100]
 mean_v=400+200*dt/2000;assert(mean_v>400&&abs(mean_v-400-200*dt/2000)<1e-10);checks=checks+1;
end
groups.M7=groups.M7+checks-mark;mark=checks;
% Extra minimum M1..M7 boundary cases frozen by planning review.
for field=1:6
 q=p;q.identity(field)=0;if field==2,q.identity(field)=3;elseif field==6,q.identity(field)=2;end
 a=map(1000,[300 300],90);a.identity=q.identity;a.map_identity=q.identity(1:3);[s,o,r]=observe(q,struct(),a);assert(r.status==2&&~o.valid);checks=checks+1;
end
for value=[-1 5.01 Inf]
 q=p;q.stop_v=value;[s,o,r]=observe(q,struct(),map(1000,[300 300],90));assert(r.status==2&&~o.valid);checks=checks+1;
end
for value=[-.01 2.01 NaN]
 q=p;q.stop_w=value;[s,o,r]=observe(q,struct(),map(1000,[300 300],90));assert(r.status==2&&~o.valid);checks=checks+1;
end
q=p;q.max_age=1000;a=map(1000,[300 300],90);[s,~,~]=observe(q,struct(),a);[s,o,r]=observe(q,s,advance(a,20));assert(r.status==1&&o.valid);checks=checks+1;
q=p;q.signature=2;q.identity(6)=1;a=map(1000,[300 300],90);[s,~,~]=observe(p,struct(),a);a.identity=q.identity;[s,o,r]=observe(q,s,a);assert(r.status==0&&~o.valid&&s.identity(6)==1);checks=checks+1;
groups.M1=groups.M1+checks-mark;mark=checks;
% Zero is a valid source timestamp; begin/stop/source may cross uint32 zero.
a=map(4294967280,[300 300],-179);a.begin=4294967200;b=advance(a,16);[s,~,~]=observe(p,struct(),a);[s,o,r]=observe(p,s,b);assert(r.status==0&&s.latest.ms==0);checks=checks+1;
b=advance(a,20);b.h=179;b.stop=1;b.stop_ms=0;b.stop_run=1;[s,o,r]=observe(p,s,b);assert(r.status==1&&o.stopped&&o.omega==-100);checks=checks+1;
b=advance(b,20);b.begin=0;[s,o,r]=observe(p,s,b);assert(r.status==0&&s.latest.begin==0);checks=checks+1;
groups.M2=groups.M2+checks-mark;mark=checks;
% Current source health: every wheel boundary, future, and before-begin value.
for wheel=1:4
 for badkind=1:3
  a=map(1400,[300 300],90);a.begin=900;[s,~,~]=observe(p,struct(),a);b=advance(a,20);
  b.wheel(wheel)=u32(b.now-399);[~,o,r]=observe(p,s,b);assert(r.status==1&&o.valid);checks=checks+1;
  if badkind==1,b.wheel(wheel)=u32(b.now-400);elseif badkind==2,b.wheel(wheel)=u32(b.now+1);else,b.wheel(wheel)=899;end
  [next,o,r]=observe(p,s,b);assert(r.status==2&&~o.valid&&isempty(fieldnames(next)));checks=checks+1;
 end
end
for badkind=1:3
 a=map(1000,[300 300],90);a.begin=800;[s,~,~]=observe(p,struct(),a);b=advance(a,20);b.angle=u32(b.now-99);[~,o,r]=observe(p,s,b);assert(r.status==1&&o.valid);checks=checks+1;
 if badkind==1,b.angle=u32(b.now-100);elseif badkind==2,b.angle=u32(b.now+1);else,b.angle=799;end
 [next,o,r]=observe(p,s,b);assert(r.status==2&&~o.valid&&isempty(fieldnames(next)));checks=checks+1;
end
for age=[100 101]
 a=map(1000,[300 300],90);[s,~,~]=observe(p,struct(),a);b=advance(a,20);b.now=b.ms+age;b.angle=b.now;b.wheel(:)=b.now;b.age=0;
 [s,o,r]=observe(p,s,b);assert((age==100&&r.status==1&&o.valid)||(age==101&&r.status==2&&~o.valid&&isempty(fieldnames(s))));checks=checks+1;
end
groups.M3=groups.M3+checks-mark;mark=checks;
% WAIT duplicates, short-anchor conflict/rollback and all metadata classes.
a=map(1000,[300 300],90);[s,~,~]=observe(p,struct(),a);[same,o,r]=observe(p,s,a);assert(r.status==0&&~o.valid&&isequal(same,s));checks=checks+1;
b=advance(a,19);[s,~,~]=observe(p,s,b);bad=b;bad.raw(1)=301;[same,o,r]=observe(p,s,bad);assert(r.status==2&&~o.valid&&isequal(same,s)&&s.anchor.ms==1000);checks=checks+1;
bad=b;bad.ms=1018;bad.updates=0;[same,o,r]=observe(p,s,bad);assert(r.status==2&&isequal(same,s));checks=checks+1;
bad=b;bad.updates=0;[s,o,r]=observe(p,s,bad);assert(r.status==0&&s.anchor.ms==1019);checks=checks+1;
a=map(1000,[300 300],90);b=advance(a,20);[s,~,~]=observe(p,struct(),a);[s,old,~]=observe(p,s,b);
for field={'corr','source','scope','nav','direction','flags','epoch','published','age','wheel','angle'}
 bad=b;name=field{1};bad.(name)=bad.(name)+1;bad.now=1021;[same,o,r]=observe(p,s,bad);old.now=1021;assert(r.status==1&&isequal(o,old)&&isequal(same,s));checks=checks+1;
end
groups.M4=groups.M4+checks-mark;mark=checks;
% Component range matches Reference; no stricter norm-v limit is introduced.
a=map(1000,[300 300],90);[s,~,~]=observe(p,struct(),a);b=advance(a,20);b.raw=a.raw+[200 200];b.xy=b.raw;[~,o,r]=observe(p,s,b);assert(r.status==1&&all(o.v==10000));checks=checks+1;
for value=[10000.1 -10000.1]
 b=advance(a,20);b.raw=a.raw+[value*.02 0];b.xy=b.raw;[same,o,r]=observe(p,s,b);assert(r.status==2&&~o.valid&&isequal(same,s));checks=checks+1;
end
for value=[3600 3600.1 -3600 -3600.1]
 b=advance(a,20);b.h=a.h+value*.02;[same,o,r]=observe(p,s,b);assert((abs(value)<=3600&&r.status==1)||(abs(value)>3600&&r.status==2&&~o.valid&&isequal(same,s)));checks=checks+1;
end
groups.M5=groups.M5+checks-mark;mark=checks;
% Cached stopped is never refreshed using a replacement stop timestamp.
a=map(1000,[300 300],90);b=advance(a,20);b.stop=1;b.stop_ms=1000;b.stop_run=1;[s,~,~]=observe(p,struct(),a);[s,o,r]=observe(p,s,b);assert(r.status==1&&o.stopped);checks=checks+1;
for kind=1:5
 bad=b;bad.now=1101;bad.angle=1101;bad.wheel(:)=1101;bad.stop_ms=1101;
 if kind==2,bad.now=1020;bad.angle=1020;bad.wheel(:)=1020;bad.stop_ms=1021;elseif kind==3,bad.now=1020;bad.angle=1020;bad.wheel(:)=1020;bad.stop_run=2;elseif kind==4,bad.now=1020;bad.angle=1020;bad.wheel(:)=1020;bad.stop=0;elseif kind==5,bad.now=1020;bad.angle=1020;bad.wheel(:)=1020;bad.stop_ms=899;end
 [same,o,r]=observe(p,s,bad);assert(r.status==2&&~o.valid&&isequal(same,s));checks=checks+1;
end
for age=[100 101]
 a=map(1000,[300 300],90);a.begin=800;b=advance(a,20);b.stop=1;b.stop_ms=b.now-age;b.stop_run=1;[s,~,~]=observe(p,struct(),a);[s,o,r]=observe(p,s,b);assert(r.status==1&&logical(o.stopped)==(age==100));checks=checks+1;
end
groups.M6=groups.M6+checks-mark;mark=checks;
% A real raw trajectory through observe exposes the finite-window deceleration lag.
for dt=[20 50 100]
 a=map(1000,[300 300],90);b=advance(a,dt);seconds=dt/1000;acceleration=-200;current_v=400;
 initial_v=current_v-acceleration*seconds;b.raw=a.raw+[initial_v*seconds+.5*acceleration*seconds^2 0];b.xy=b.raw;
 [s,~,~]=observe(p,struct(),a);[s,o,r]=observe(p,s,b);expected=.5*(initial_v+current_v);
 assert(r.status==1&&abs(o.v(1)-expected)<1e-8&&o.v(1)>current_v);checks=checks+1;
end
groups.M7=groups.M7+checks-mark;mark=checks;
for field=1:7
 for invalid=[NaN Inf -Inf]
  a=map(1000,[300 300],90);[s,~,~]=observe(p,struct(),a);b=advance(a,20);
  if field<=2,b.raw(field)=invalid;elseif field<=4,b.xy(field-2)=invalid;elseif field<=6,b.corr(field-4)=invalid;else,b.h=invalid;end
  [next,o,r]=observe(p,s,b);assert(r.status==2&&~o.valid&&isempty(fieldnames(next)));checks=checks+1;
 end
end
a=map(1000,[3e38 300],90);b=advance(a,20);b.raw(1)=-3e38;b.xy=b.raw;[s,~,~]=observe(p,struct(),a);[same,o,r]=observe(p,s,b);assert(r.status==2&&~o.valid&&isequal(same,s));checks=checks+1;
groups.M5=groups.M5+checks-mark;mark=checks;
a=map(1000,[300 300],90);b=advance(a,20);[s,~,~]=observe(p,struct(),a);[s,~,~]=observe(p,s,b);
bad=b;bad.raw(1)=bad.raw(1)+1;bad.updates=0;[same,o,r]=observe(p,s,bad);assert(r.status==2&&~o.valid&&isequal(same,s));checks=checks+1;
groups.M4=groups.M4+checks-mark;mark=checks;
% Numeric-fix precondition: no new velocity limit, including large anchors.
for anchor=[0 300 -300 1e8 -1e8 3e38 -3e38]
 for requested=[-200 0 200]
  a=map(1000,[double(single(anchor)) 300],90);b=advance(a,20);b.raw(1)=double(single(a.raw(1)+requested));b.xy=b.raw;
  exact=(b.raw(1)-a.raw(1))*50;[s,~,~]=observe(p,struct(),a);[s,o,r]=observe(p,s,b);
  % Huge anchors may represent only zero displacement; no physical claim.
  assert(abs(exact)<=10000&&r.status==1&&abs(o.v(1)-exact)<1e-8);checks=checks+1;
  upper=single(single(a.raw(1))+single(200));lower=single(single(a.raw(1))-single(200));
  assert(~(single(b.raw(1))>upper||single(b.raw(1))<lower));checks=checks+1;
 end
end
for sign_value=[-1 1]
 a=map(1000,[300 300],90);b=advance(a,20);b.raw(1)=a.raw(1)+sign_value*200.01;b.xy=b.raw;[s,~,~]=observe(p,struct(),a);[same,o,r]=observe(p,s,b);
 assert(r.status==2&&~o.valid&&isequal(same,s));checks=checks+1;
end
groups.M5=groups.M5+checks-mark;mark=checks;
assert(sum(struct2array(groups))==checks);

writematrix(vectors,fullfile(out,'adapter_vectors_v9.csv'));writematrix(noise,fullfile(out,'quantization_scan_v9.csv'));
summary=struct('status','PASS','checks',checks,'groups',groups,'vectors',size(vectors,1),'matlab_version',version,'scope','Pure copied AppMapPose finite-window ESTIMATED feedback; no caller, instantaneous physical feedback, physical stop or deployment proof','nominal_min_dt_ms',20,'max_dt_ms',100,'quantum_mm',quantum,'model_only',true,'hardware_accessed',false);
f=fopen(fullfile(out,'matlab_summary_v9.json'),'w');fprintf(f,'%s',jsonencode(summary));fclose(f);disp(summary);
end
function p=plan()
p=struct('valid',1,'signature',1,'identity',[1 1 1 1 1 0],'model_only',1,'count',2,'box_count',0,'max_age',100,'stop_v',2,'stop_w',.5);
end
function yes=id_ok(id),yes=id(1)>0&&(id(2)==1||id(2)==2)&&all(id(3:5)>0)&&(id(6)==0||id(6)==1);end
function h=wrap(h),h=mod(h+180,360)-180;end
function n=u32(n),n=mod(n,4294967296);end
function d=delta(a,b),d=u32(a-b);end
function yes=not_before(a,b),yes=delta(a,b)<2147483648;end
function a=map(ms,xy,h)
a=struct('ms',ms,'now',ms,'begin',900,'published',ms,'age',0,'updates',1,'identity',[1 1 1 1 1 0],'map_identity',[1 1 1],...
 'raw',xy,'xy',xy,'corr',[0 0],'h',h,'valid',1,'state',2,'based',1,'continuity',0,'reasons',0,'local',0,'wheel',repmat(ms,1,4),'angle',ms,...
 'source',1,'scope',0,'nav',0,'direction',0,'flags',0,'epoch',0,'stop',0,'stop_ms',0,'stop_run',0);
end
function b=advance(a,dt)
b=a;b.ms=u32(a.ms+dt);b.now=b.ms;b.published=b.ms;b.age=0;b.updates=a.updates+1;b.wheel(:)=b.ms;b.angle=b.ms;
end
function o=zero()
o=struct('identity',zeros(1,6),'valid',0,'ms',0,'now',0,'xy',[0 0],'h',0,'v',[0 0],'omega',0,'stopped',0,'source',0);
end
function s=prime(p,a,o)
s=struct('signature',p.signature,'identity',p.identity,'anchor',struct('ms',a.ms,'raw',a.raw,'h',a.h),'latest',core(a),'updates',a.updates,'ready',false,'cached',o,'stop_ms',0,'stop_run',0);
end
function c=core(a)
c=struct('ms',a.ms,'begin',a.begin,'raw',a.raw,'xy',a.xy,'h',a.h);
end
function yes=stop_ok(p,a,ms,run)
yes=run==p.identity(1)&&not_before(a.now,ms)&&not_before(ms,a.begin)&&delta(a.now,ms)<=p.max_age;
end
function [s,o,r]=observe(p,s,a)
o=zero();r=struct('status',2,'stationary',false,'stop_pending',false,'source',a.source,'scope',a.scope,'nav',a.nav,'direction',a.direction,'flags',a.flags,'epoch',a.epoch,'corr',a.corr,'mixed_verified',false,'dt',0);
if ~id_ok(p.identity)||p.valid~=1||p.signature==0||p.count<2||p.count>32||p.box_count>32||p.model_only~=1||p.max_age<100||p.max_age>1000||~all(isfinite([p.stop_v p.stop_w]))||p.stop_v<0||p.stop_v>5||p.stop_w<0||p.stop_w>2,return;end
if ~isequal(a.identity,p.identity)||~isequal(a.map_identity,p.identity(1:3)),return;end
% Do not let an old pose plus a low update count rewind the anchor.
if ~isempty(fieldnames(s))&&s.signature==p.signature&&isequal(s.identity,p.identity)&&a.begin==s.latest.begin&&~not_before(a.ms,s.latest.ms),return;end
if ~not_before(a.now,a.ms)||~not_before(a.ms,a.begin)||any(~arrayfun(@(x)not_before(a.now,x)&&not_before(x,a.begin),[a.wheel a.angle])),s=struct();return;end
if ~all(isfinite([a.raw a.xy a.corr a.h]))||abs(a.h)>3600||a.valid~=1||a.state~=2||a.based~=1||a.continuity||a.reasons||a.local||delta(a.now,a.ms)>p.max_age||delta(a.now,a.angle)>=100||any(arrayfun(@(x)delta(a.now,x)>=400,a.wheel))
 s=struct();return;
end
stop=a.stop==1&&stop_ok(p,a,a.stop_ms,a.stop_run);
if isempty(fieldnames(s))||s.signature~=p.signature||~isequal(s.identity,p.identity)||a.begin~=s.latest.begin
 s=prime(p,a,o);r.status=0;return;
end
if ~not_before(a.ms,s.latest.ms),return;end
if a.ms==s.latest.ms&&~isequal(core(a),s.latest),return;end
if a.updates<s.updates,s=prime(p,a,o);r.status=0;return;end
if a.ms==s.latest.ms
 r.stop_pending=(stop~=logical(s.cached.stopped))||(stop&&a.stop_ms~=s.stop_ms);
 r.status=0;
 if s.ready
  if s.cached.stopped&&(~stop||~stop_ok(p,a,s.stop_ms,s.stop_run)),r.status=2;return;end
  o=s.cached;o.now=a.now;r.status=1;r.stationary=norm(o.v)<=p.stop_v&&abs(o.omega)<=p.stop_w;
 end
 return;
end
dt=delta(a.ms,s.anchor.ms);r.dt=dt;
if dt>100,s=prime(p,a,o);r.status=0;return;end
next=s;next.latest=core(a);next.updates=a.updates;next.ready=false;r.status=0;
if dt<20,s=next;return;end
% Equivalent existing component domain, before an overflowing float subtraction.
if any(a.raw>s.anchor.raw+10*dt)||any(a.raw<s.anchor.raw-10*dt),r.status=2;return;end
v=(a.raw-s.anchor.raw)*1000/dt;omega=wrap(a.h-s.anchor.h)*1000/dt;
if ~all(isfinite([v omega]))||any(abs(v)>10000)||abs(omega)>3600,r.status=2;return;end
o=struct('identity',p.identity,'valid',1,'ms',a.ms,'now',a.now,'xy',a.xy,'h',a.h,'v',v,'omega',omega,'stopped',double(stop),'source',1);
next.anchor=struct('ms',a.ms,'raw',a.raw,'h',a.h);next.ready=true;next.cached=o;next.stop_ms=a.stop_ms;next.stop_run=a.stop_run;s=next;r.status=1;r.stationary=norm(v)<=p.stop_v&&abs(omega)<=p.stop_w;
end
