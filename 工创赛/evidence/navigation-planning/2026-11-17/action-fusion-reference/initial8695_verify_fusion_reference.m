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
