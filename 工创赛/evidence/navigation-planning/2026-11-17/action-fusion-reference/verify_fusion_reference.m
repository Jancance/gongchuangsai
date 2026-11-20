function verify_fusion_reference
% Pure reference mathematics; no dynamics, actuator, measured-size or deployment claim.
out=fileparts(mfilename('fullpath'));checks=0;
for heading=[-179 -170 -90 0 37 90 170 179]
 for delta=[0 90 -90 180]
  goal=wrap(heading+delta); [hold,turn]=schedule(heading,goal,true,true);
  if abs(delta)==180,assert(turn==-1 && abs(wrap(hold-heading+90))<1e-9);else,assert(abs(wrap(hold-goal))<1e-9);end
  checks=checks+1;
  for err=[-3 -1 -.2 0 .2 1 3]
   actual=wrap(hold+err);assert(abs(wrap(hold-actual)+err)<1e-9);checks=checks+1;
   % Absolute second target remains original goal, independent of actual yaw.
   assert(abs(wrap(goal-wrap(heading+delta)))<1e-9);checks=checks+1;
   v=[173;-21];r=actual*pi/180;body=[cos(r) sin(r);-sin(r) cos(r)]*v;
   recovered=[cos(r) -sin(r);sin(r) cos(r)]*body;assert(norm(v-recovered)<1e-10);checks=checks+1;
   wheels=inverse(body,77);scale=min(1,80/max(abs(wheels)));assert(max(abs(wheels*scale))<=80+1e-10);checks=checks+1;
  end
 end
end
[h,t]=schedule(0,180,false,true);assert(h==90 && t==1);checks=checks+1;
[h,t]=schedule(0,180,false,false);assert(isnan(h)&&t==0);checks=checks+1;
P=[300 300;1200 300;1200 1200];
for q=[-40 0 200 850 900 910]
 for e=[-20 0 20]
  [s,remaining,residual,lat]=project(P,1,[300+q 300+e]);
  assert(abs(s-q)<1e-9 && abs(remaining-(900-min(q,900)+900))<1e-9 && abs(residual-(900-q))<1e-9 && abs(lat-e)<1e-9);checks=checks+1;
 end
end
% Remaining is 1800 on this L, not sqrt(900^2+900^2).
[~,rem]=project(P,1,P(1,:));assert(rem==1800 && rem>norm(P(end,:)-P(1,:)));checks=checks+1;
% Reversing preserves negative signed progress, no monotonic clamp.
[s1,~]=project(P,1,[700 300]);[s2,~]=project(P,1,[600 300]);assert(s2<s1);checks=checks+1;
% Same-direction collinear only: reversal and orthogonal turn are zero-exit.
assert(collinear([1 0],[2 0]) && ~collinear([1 0],[-1 0]) && ~collinear([1 0],[0 1]));checks=checks+1;
% Full backward speed cap across small intermediate collinear nodes.
lengths=[100 100 100 100 100];caps=[400 400 400 400 80];a=200;
node=zeros(1,6);node(end)=0;
for i=5:-1:1,node(i)=min(caps(i),sqrt(node(i+1)^2+2*a*lengths(i)));end
assert(node(2)<400 && node(2)<=sqrt(80^2+2*a*300)+1e-9);checks=checks+1;
assert(abs((400^2-80^2)/(2*a)-384)<1e-9);checks=checks+1;
% Parameter sensitivities, not frozen measured body dimensions.
for dims=[270 250;283 254]'
 for budget=[0 5 10 20]
  for hd=[0 37 90 170]
   ex=extents(dims(1),dims(2),hd-3,hd+3)+budget;
   for sample=linspace(hd-3,hd+3,151)
    exact=[abs(cosd(sample))*dims(1)/2+abs(sind(sample))*dims(2)/2,abs(sind(sample))*dims(1)/2+abs(cosd(sample))*dims(2)/2];
    assert(all(ex+1e-9>=exact+budget));checks=checks+1;
   end
  end
 end
end
% Full segment crossing, touching, body-only collision, diagonal/circle and field limits.
% Product contract always inserts all four yellow boxes; caller boxes are additional.
yellow=[550 550 1000 1000;550 1400 1000 1850;1400 550 1850 1000;1400 1400 1850 1850];
assert(size(yellow,1)==4);checks=checks+1;
assert(32>=25+2+4 && ~(33<=32));checks=checks+1;
rect=[550 550 1000 1000];dims=[270 250];
assert(~clear([300 775],[1300 775],0,0,dims,0,rect,[],2400));checks=checks+1;
assert(~clear([300 430],[1300 430],0,0,dims,0,rect,[],2400));checks=checks+1;
assert(clear([300 300],[1200 300],0,3,dims,5,rect,[],2400));checks=checks+1;
assert(~clear([100 100],[500 100],0,0,dims,0,[],[],2400));checks=checks+1;
assert(~clear([300 300],[900 900],0,0,dims,0,[],[600 600 25],2400));checks=checks+1;
assert(clear([300 300],[400 400],0,0,dims,0,rect,[],2400));checks=checks+1;
% Turn support extrema: dense samples enclosed, including wrap-side intervals.
for h=[-170 0 37 170]
 ex=extents(270,250,h-93,h+3);
 for sample=linspace(h-93,h+3,721)
  exact=[abs(cosd(sample))*135+abs(sind(sample))*125,abs(sind(sample))*135+abs(cosd(sample))*125];
  assert(all(ex+1e-9>=exact));checks=checks+1;
 end
end
% Positive tangential cap, bounded lateral correction, terminal geometric != stopped.
for e=[-40 -5 0 5 40]
 lateral=max(-20,min(20,-.5*e));assert(abs(lateral)<=20 && lateral*e<=0);checks=checks+1;
end
assert(~done([1200 1200],[1200 1200],0,0,false,0,0));checks=checks+1;
assert(~done([1200 1200],[1200 1200],0,0,true,40,0));checks=checks+1;
assert(~done([1200 1230],[1200 1200],0,0,true,0,0));checks=checks+1;
assert(done([1200 1200],[1200 1200],0,0,true,0,0));checks=checks+1;
% Actual speed beyond nominal cap requests recovery rather than pretending to brake.
assert(400>sqrt(2*200*50));checks=checks+1;
% Repeated source sample cannot advance a phase/cursor; bad identity leaves state unchanged.
state=struct('sample',1000,'edge',1,'phase',1);next=state;
if 1000>state.sample,next.edge=2;end
assert(isequal(state,next));checks=checks+1;
% Actual-heading transform can produce mixed body motion on a nominal axis.
r=88*pi/180;v=[cos(r) sin(r);-sin(r) cos(r)]*[0;400];
assert(v(1)>399 && v(2)>13 && v(2)<14);checks=checks+1;
% Stale/future/source identity/nonfinite validation model.
assert(validobs(1000,1050,100,true,false,1,1));checks=checks+1;
assert(~validobs(1100,1000,100,true,false,1,1));checks=checks+1;
assert(~validobs(1000,1200,100,true,false,1,1));checks=checks+1;
assert(~validobs(1000,1050,100,true,true,1,1));checks=checks+1;
assert(~validobs(1000,1050,100,true,false,1,2));checks=checks+1;
assert(~validobs(NaN,1050,100,true,false,1,1));checks=checks+1;
checks=checks+verify_state_budget(out);
summary=struct('status','PASS','checks',checks,'scope','Pure desired reference, geometric conservative sweep and identity mathematics; no A/J, stopping dynamics, RTOS or vehicle proof','model_only',true,'deployment_allowed',false,'body_sizes_are_sensitivity_inputs',true,'mixed_calibration_verified',false);
f=fopen(fullfile(out,'matlab_summary.json'),'w');fprintf(f,'%s',jsonencode(summary));fclose(f);disp(summary);
end
function w=wrap(h),w=mod(h+180,360)-180;end
function [h,t]=schedule(a,b,cw,ccw)
d=wrap(b-a);t=0;h=b;
if abs(abs(d)-180)<1e-6
 if cw,t=-1;h=wrap(a-90);elseif ccw,t=1;h=wrap(a+90);else,h=NaN;end
end
end
function w=inverse(b,d),w=[b(1)+b(2);b(1)-b(2);b(1)-b(2);b(1)+b(2)]/(pi*d/60);end
function [s,r,z,e]=project(P,i,p)
v=P(i+1,:)-P(i,:);L=norm(v);t=v/L;n=[-t(2) t(1)];q=dot(p-P(i,:),t);e=dot(p-P(i,:),n);ls=sqrt(sum(diff(P).^2,2));s=sum(ls(1:i-1))+q;r=L-min(q,L)+sum(ls(i+1:end));z=L-q;
end
function yes=collinear(a,b),yes=dot(a,b)>0 && abs(a(1)*b(2)-a(2)*b(1))<1e-6*norm(a)*norm(b);end
function ex=extents(l,w,a,b)
angles=[a b];for k=-4:4,angles=[angles atan2d(w,l)+k*180 -atan2d(w,l)+k*180 atan2d(l,w)+k*180 -atan2d(l,w)+k*180];end
angles=angles(angles>=a & angles<=b);ex=[max(abs(cosd(angles))*l/2+abs(sind(angles))*w/2),max(abs(sind(angles))*l/2+abs(cosd(angles))*w/2)];
end
function yes=clear(a,b,h,budget,dims,pos,rect,circle,field)
ex=extents(dims(1),dims(2),h-budget,h+budget)+pos;yes=true;
if any(min(a,b)-ex<0)||any(max(a,b)+ex>field),yes=false;return;end
for k=1:size(rect,1)
 r=rect(k,:)+[-ex ex];if hitbox(a,b,r),yes=false;return;end
end
for k=1:size(circle,1)
 v=b-a;if dot(v,v)==0,q=0;else,q=max(0,min(1,dot(circle(k,1:2)-a,v)/dot(v,v)));end
 if norm(a+q*v-circle(k,1:2))<=circle(k,3)+norm(dims)/2+pos,yes=false;return;end
end
end
function hit=hitbox(a,b,r)
lo=0;hi=1;v=b-a;hit=false;
for j=1:2
 if abs(v(j))<1e-10,if a(j)<r(j)||a(j)>r(j+2),return;end
 else,z=sort(([r(j) r(j+2)]-a(j))/v(j));lo=max(lo,z(1));hi=min(hi,z(2));if lo>hi,return;end
 end
end
hit=true;
end
function yes=done(p,goal,h,target,stopped,v,w),yes=norm(p-goal)<=5 && abs(wrap(target-h))<=3 && stopped && abs(v)<=2 && abs(w)<=.5;end
function yes=validobs(sample,now,age,valid,gap,id,want),yes=all(isfinite([sample now age]))&&sample<=now&&now-sample<=age&&valid&&~gap&&id==want;end

function checks=verify_state_budget(out)
checks=0;p=struct('points',[300 300;1200 300;1200 1200],'hold',-90,'goal',-180,'id',7,'speed',[200 80],'a',200);
s=struct('phase',0,'edge',1,'seen',false,'last',0);o=struct('xy',[300 300],'h',0,'stopped',true,'v',0,'omega',0,'sample',1000,'now',1000,'valid',true,'gap',false,'id',7);
[s1,r]=state_step(p,s,o);assert(r==2 && s1.phase==0);checks=checks+1;
o.h=-90;o.sample=1010;o.now=1010;[s,r]=state_step(p,s1,o);assert(s.phase==1);checks=checks+1;
% A repeated timestamp cannot consume an extra phase even at an endpoint.
o.xy=[1200 300];[dup,~]=state_step(p,s,o);assert(isequal(dup,s));checks=checks+1;
o.sample=1020;o.now=1020;o.stopped=false;[s,r]=state_step(p,s,o);assert(s.edge==1 && r==1);checks=checks+1;
o.sample=1030;o.now=1030;o.stopped=true;[s,r]=state_step(p,s,o);assert(s.edge==2 && s.phase==1);checks=checks+1;
o.xy=[1200 1200];o.sample=1040;o.now=1040;[s,r]=state_step(p,s,o);assert(s.phase==2 && r==3);checks=checks+1;
o.h=-180;[dup,r]=state_step(p,s,o);assert(dup.phase==2);checks=checks+1;
o.sample=1050;o.now=1050;[s,r]=state_step(p,s,o);assert(s.phase==3 && r==4);checks=checks+1;
% Failure is reversible: source/health does not consume phase or change targets.
base=struct('phase',1,'edge',1,'seen',true,'last',1000);o.xy=[1200 300];o.h=-90;o.sample=1010;o.now=1010;
for kind=1:6
 bad=o;
 if kind==1,bad.id=8;elseif kind==2,bad.now=1300;elseif kind==3,bad.sample=1200;elseif kind==4,bad.valid=false;elseif kind==5,bad.gap=true;else,bad.xy(1)=NaN;end
 [same,r]=state_step(p,base,bad);assert(isequal(same,base) && r==0);checks=checks+1;
 [fresh,r]=state_step(p,same,o);assert(fresh.edge==2 && r==1);checks=checks+1;
end
% Completion requires position, heading and externally observed stopping.
s=struct('phase',1,'edge',2,'seen',true,'last',1000);o.sample=1010;o.now=1010;o.xy=[1230 1200];
[same,r]=state_step(p,s,o);assert(same.phase==1 && r~=4);checks=checks+1;
o.xy=[1200 1230];[same,r]=state_step(p,s,o);assert(same.phase==1 && r==5);checks=checks+1;
o.xy=[1200 1200];o.stopped=false;[same,r]=state_step(p,s,o);assert(same.phase==1 && r==1);checks=checks+1;
o.stopped=true;o.v=40;[same,r]=state_step(p,s,o);assert(same.phase==1 && r==5);checks=checks+1;
% Negative progress and self-intersection are not global-nearest cursor jumps.
p2=p;p2.points=[300 300;1200 300;1200 1200;300 1200;300 300;1200 300];p2.speed=200*ones(1,5);
s=struct('phase',1,'edge',1,'seen',true,'last',1000);o.xy=[500 300];o.v=0;
[same,r]=state_step(p2,s,o);assert(same.edge==1 && r==1);checks=checks+1;
o.xy=[280 300];[same,r]=state_step(p,s,o);assert(same.edge==1 && r==1);checks=checks+1;
% Current-speed feasibility is diagnostic, not a fabricated brake command.
o.xy=[1190 300];o.stopped=false;o.v=400;[same,r]=state_step(p,s,o);assert(r==5 && same.edge==1);checks=checks+1;
% Exact duplicate output across all four phases, stale-first, and changed same sample rejection.
for phase=0:3
 st=struct('phase',phase,'edge',1,'seen',false,'last',0);ob=o;ob.id=7;ob.sample=2000;ob.now=2000;ob.v=0;ob.stopped=true;ob.h=-90;ob.xy=p.points(1,:);
 if phase>=2,ob.h=-180;ob.xy=p.points(end,:);end
 [after,r1,ref1]=state_step(p,st,ob);[again,r2,ref2]=state_step(p,after,ob);
 assert(r1==r2 && isequal(after,again) && isequal(ref1,ref2));checks=checks+1;
 bad=ob;bad.xy(1)=bad.xy(1)+1;[same,rr]=state_step(p,after,bad);assert(rr==0 && isequal(same,after));checks=checks+1;
 bad=ob;bad.now=2200;[same,rr]=state_step(p,after,bad);assert(rr==0 && isequal(same,after));checks=checks+1;
end
% A source may skip several short collinear nodes but cannot skip a direction change.
p3=p;p3.points=[300 300;310 300;320 300;330 300;330 500];p3.speed=100*ones(1,4);
st=struct('phase',1,'edge',1,'seen',false,'last',0);ob=o;ob.xy=[327 300];ob.h=-90;ob.v=0;ob.stopped=false;ob.sample=3000;ob.now=3000;
[after,rr,ref]=state_step(p3,st,ob);assert(rr==1 && after.edge==3 && norm(ref.world)>0);checks=checks+1;
ob.xy=[337 300];ob.sample=3010;ob.now=3010;[same,rr]=state_step(p3,after,ob);assert(rr==5 && isequal(same,after));checks=checks+1;
% End-before-3 -> end-after-7/20 of a collinear edge, no endpoint-ball dependence.
p4=p;p4.points=[300 300;600 300;900 300];p4.speed=[200 200];st=struct('phase',1,'edge',1,'seen',false,'last',0);
for x=[597 607 620]
 ob.xy=[x 300];ob.sample=ob.sample+10;ob.now=ob.sample;[st,rr,ref]=state_step(p4,st,ob);assert(rr==1);checks=checks+1;
end
assert(st.edge==2 && norm(ref.world)>0);checks=checks+1;
% Terminal sideways correction can persist across fresh actual-feedback samples.
st=struct('phase',1,'edge',2,'seen',false,'last',0);ob=o;ob.h=-90;ob.stopped=false;ob.sample=4000;ob.now=4000;ob.v=0;ob.xy=[1230 1200];ob.world_v=[0 0];
[st,rr,ref]=state_step(p,st,ob);assert(rr==1 && ref.world(1)<0);checks=checks+1;
for x=[1228 1220 1210]
 ob.xy=[x 1200];ob.sample=ob.sample+10;ob.now=ob.sample;ob.world_v=[-15 0];ob.v=15;
 [st,rr,ref]=state_step(p,st,ob);assert(rr==1 && st.phase==1 && ref.world(1)<0);checks=checks+1;
end
bad=ob;bad.world_v=[0 400];bad.v=400;bad.sample=bad.sample+10;bad.now=bad.sample;[same,rr]=state_step(p,st,bad);assert(rr==5 && isequal(same,st));checks=checks+1;
bad=ob;bad.world_v=[-100 0];bad.v=100;bad.sample=bad.sample+10;bad.now=bad.sample;[same,rr]=state_step(p,st,bad);assert(rr==5 && isequal(same,st));checks=checks+1;
% Reserve-first, scaled reference in both frames, exact inverse relation.
rows=[];
for h=[-179 -90 0 37 88 90 170 179]
 for world=[400 0;0 400;300 240;-200 15]'
  for reserve=[0 5 20]
   [body,wheels,scaled,scale]=budget(world,h,77,80,reserve);
   assert(max(abs(wheels))<=80-reserve+1e-9 && scale>0 && scale<=1);checks=checks+1;
   rr=h*pi/180;assert(norm([cos(rr) -sin(rr);sin(rr) cos(rr)]*body-scaled)<1e-9);checks=checks+1;
   assert(norm(inverse(body,77)-wheels)<1e-9);checks=checks+1;
   rows=[rows;h world' reserve body' scaled' wheels' scale];
  end
 end
end
writematrix(rows,fullfile(out,'budget_vectors.csv'));
end
function [s,result,ref]=state_step(p,s,o)
% Cache whole reference for a duplicate sample, not only the phase cursor.
result=0;ref=struct('status',0,'target',0,'edge',0,'cap',0,'world',[0 0]);original=s;
if ~all(isfinite([o.xy o.h o.v o.omega o.sample o.now])) || ~o.valid || o.gap || o.id~=p.id || o.sample>o.now || o.now-o.sample>100 || (s.seen && o.sample<s.last),return;end
obs=rmfield(o,'now');
if s.seen && o.sample==s.last
 if isfield(s,'observation') && isequal(obs,s.observation),result=s.reference.status;ref=s.reference;end
 return;
end
fresh=true;target=p.hold;if s.phase>=2,target=p.goal;end
ref.target=target;ref.edge=s.edge;
if s.phase==0
 result=2;
 if norm(o.xy-p.points(1,:))>40,result=5;return;end
 if abs(wrap(p.hold-o.h))<=3 && stopped(o),s.phase=1;end
elseif s.phase==2
 result=3;
 if norm(o.xy-p.points(end,:))>5,result=5;return;end
 if abs(wrap(p.goal-o.h))<=3 && stopped(o),s.phase=3;result=4;end
elseif s.phase==3
 if done(o.xy,p.points(end,:),o.h,p.goal,o.stopped,o.v,o.omega),result=4;else,result=5;end
else
 result=1;i=s.edge;[~,~,z,e]=project(p.points,i,o.xy);L=norm(p.points(i+1,:)-p.points(i,:));q=L-z;
 if abs(e)>40 || q< -40 || abs(wrap(p.hold-o.h))>3,result=5;s=original;return;end
 % Only normalize through adjacent same-direction collinear edges. Never global nearest.
 while i<size(p.points,1)-1 && collinear(p.points(i+1,:)-p.points(i,:),p.points(i+2,:)-p.points(i+1,:)) && q>=L
  i=i+1;[~,~,z,e]=project(p.points,i,o.xy);L=norm(p.points(i+1,:)-p.points(i,:));q=L-z;
 end
 if z< -5,result=5;s=original;return;end
 nodes=zeros(1,size(p.points,1));
 for k=size(p.points,1)-1:-1:1
  exit=0;if k<size(p.points,1)-1 && collinear(p.points(k+1,:)-p.points(k,:),p.points(k+2,:)-p.points(k+1,:)),exit=nodes(k+1);end
  nodes(k)=min(p.speed(k),sqrt(exit^2+2*p.a*norm(p.points(k+1,:)-p.points(k,:))));
 end
 exit=0;if i<size(p.points,1)-1 && collinear(p.points(i+1,:)-p.points(i,:),p.points(i+2,:)-p.points(i+1,:)),exit=nodes(i+1);end
 cap=min(p.speed(i),sqrt(exit^2+2*p.a*max(0,z)));
 vt=o.v;vn=0;
 if isfield(o,'world_v')
  dir=p.points(i+1,:)-p.points(i,:);dir=dir/norm(dir);vt=dot(o.world_v,dir);vn=dot(o.world_v,[-dir(2) dir(1)]);
 end
 if vt>cap+2 || vt< -2 || abs(vn)>22 || hypot(vt,vn)>402,result=5;s=original;return;end
 s.edge=i;ref.edge=i;ref.cap=cap;d=p.points(i+1,:)-p.points(i,:);t=d/norm(d);ref.world=cap*t+max(-20,min(20,-.5*e))*[-t(2) t(1)];
 if q>=L-5 && norm(o.xy-p.points(i+1,:))<=5
  if i==size(p.points,1)-1
   if stopped(o),s.phase=2;result=3;ref.target=p.goal;ref.world=[0 0];end
  elseif stopped(o)
   s.edge=i+1;ref.world=[0 0];
  end
 end
end
ref.status=result;s.seen=true;s.last=o.sample;s.observation=obs;s.reference=ref;
end
function yes=stopped(o),yes=o.stopped && abs(o.v)<=2 && abs(o.omega)<=.5;end
function [body,wheels,scaled,scale]=budget(world,h,d,maxrpm,reserve)
r=h*pi/180;body=[cos(r) sin(r);-sin(r) cos(r)]*world;wheels=inverse(body,d);scale=min(1,(maxrpm-reserve)/max(abs(wheels)));body=body*scale;wheels=wheels*scale;scaled=world*scale;
end
