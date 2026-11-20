function verify_map_nav
% Engineering candidate only. Segment net raw displacement, no target injection.
out=fileparts(mfilename('fullpath'));checks=0;
cross=[-.00753449552346 .0179055385 -.0110117302776 .00614032994653];
main=[-.00281456044014 -.00670232061578 -.00634297364924 -.00457267364223];
for direction=0:3
 for angle=[-181 -90 0 90 179 270]
  for distance=[0 1 300 1500 2100 -20]
   s=fresh;v=axis(direction,angle);a=accepted(1,110,direction,angle,2);
   s=step(s,[22 11],100,a,true,true);assert(~s.based&&isequal(s.corr,[0 0]));checks=checks+1;
   s=step(s,[24 12],120,a,true,true);assert(s.based&&isequal(s.corr,[0 0]));checks=checks+1;
   origin=s.origin;raw=origin+distance*v;
   s=step(s,raw,140,a,true,true);[~,side]=basis(angle);
   if direction==1||direction==2,[side,~]=basis(angle);end
   expected=.5*max(distance,0)*(main(direction+1)*v+cross(direction+1)*side);
   assert(norm(s.corr-expected)<1e-9&&isequal(s.raw,raw));checks=checks+1;
   previous=s;s=step(s,raw,140,a,true,true);assert(isequal(s,previous));checks=checks+1;
   % Reversal follows net displacement, never incremental absolute travel.
   s=step(s,origin+0.5*max(distance,0)*v,160,a,true,true);
   assert(norm(s.corr-.5*expected)<1e-9);checks=checks+1;
   % Done freezes the last published correction; late raw tail remains raw.
   frozen=s.corr;s=step(s,origin+100*v,180,a,false,true);
   s=step(s,origin+120*v,200,a,false,true);assert(isequal(s.corr,frozen)&&s.partial);checks=checks+1;
   % New accepted segment retains history and waits for common source time.
   b=accepted(2,230,mod(direction+1,4),angle+90,1);
   s=step(s,origin+140*v,220,b,true,true);assert(~s.based&&isequal(s.corr,frozen));checks=checks+1;
   s=step(s,origin+160*v,240,b,true,true);assert(s.based&&isequal(s.corr,frozen));checks=checks+1;
   previous=s;s=step(s,[999 999],260,b,true,false);assert(isequal(s,previous));checks=checks+1;
  end
 end
end
% Multi-segment rotation/unsupported/idle preserves historical offset.
s=fresh;raw=[2250 150];a=accepted(1,100,0,90,1);
s=step(s,raw,100,a,true,true);s=step(s,raw+[0 2100],200,a,true,true);first=s.corr;
a=accepted(2,210,4,90,0);s=step(s,raw+[0 2110],220,a,true,true);
assert(isequal(s.corr,first)&&s.partial);checks=checks+1;
a=accepted(3,230,2,180,2);s=step(s,raw+[0 2120],240,a,true,true);
s=step(s,raw+[0 2220],260,a,true,true);assert(s.scope==3&&s.applied&&all(isfinite(s.corr)));checks=checks+1;
% A failed submission is represented by an unchanged accepted event.
old=s;s=step(s,s.raw,s.ms,a,true,true);assert(isequal(s,old));checks=checks+1;
% First common sample supplies actual heading; submitted/planned angle ignored.
q=fresh;b=accepted(1,110,0,90,2);
q=step(q,[150 150],120,b,true,true,87);
q=step(q,[150 150]+100*axis(0,87),140,b,true,true,86);
[front,left]=basis(87);expected=.5*100*(main(1)*front+cross(1)*left);
assert(norm(q.corr-expected)<1e-9&&q.angle==87);checks=checks+1;
% Getter preemption freezes raw and corrected pose from one published sample.
published=s;tentative=step(s,s.raw+[20 0],280,a,true,true);assert(~isequal(tentative.raw,published.raw));checks=checks+1;
s=published;assert(isequal(s.raw+s.corr,published.raw+published.corr));checks=checks+1;
% Mid-segment map new round does not inherit a pre-round accepted event.
s=fresh;a=accepted(0,0,4,0,0);s=step(s,[150 150],100,a,false,true);
assert(~s.applied&&s.scope==0&&isequal(s.corr,[0 0]));checks=checks+1;
summary=struct('checks',checks,'status','PASS','scope','Software model; no new measurements or hardware acceptance','cross',cross,'main',main);
f=fopen(fullfile(out,'matlab_summary.json'),'w');fprintf(f,'%s',jsonencode(summary));fclose(f);disp(summary);
end
function s=fresh
s=struct('epoch',0,'begin',0,'direction',4,'angle',0,'class',0,'active',false,'based',false,'origin',[0 0],'history',[0 0],'corr',[0 0],'raw',[0 0],'ms',0,'scope',0,'applied',false,'partial',false);
end
function a=accepted(epoch,begin,direction,angle,class)
a=struct('epoch',epoch,'begin',begin,'direction',direction,'angle',angle,'class',class);
end
function s=step(s,raw,ms,a,active,valid,heading)
if nargin<7,heading=a.angle;end
if ~valid,return;end
if s.epoch~=a.epoch
 s.history=s.corr;s.epoch=a.epoch;s.begin=a.begin;s.direction=a.direction;s.angle=a.angle;s.class=a.class;s.based=false;s.active=active;s.partial=true;
end
if ~active,s.history=s.corr;s.active=false;end
if s.active&&s.class&&s.direction<4&&ms>=s.begin
 if ~s.based,s.origin=raw;s.angle=heading;s.based=true;
 else
  cross=[-.00753449552346 .0179055385 -.0110117302776 .00614032994653];main=[-.00281456044014 -.00670232061578 -.00634297364924 -.00457267364223];
  v=axis(s.direction,s.angle);[front,left]=basis(s.angle);side=left;if s.direction==1||s.direction==2,side=front;end
  d=max(dot(raw-s.origin,v),0);s.corr=s.history+.5*d*(cross(s.direction+1)*side+main(s.direction+1)*v);
  if d>0,s.applied=true;s.scope=bitor(s.scope,s.class);end
 end
else
 s.partial=true;
end
s.raw=raw;s.ms=ms;
end
function [front,left]=basis(angle)
front=[cosd(angle) sind(angle)];left=[-sind(angle) cosd(angle)];
end
function v=axis(direction,angle)
[front,left]=basis(angle);v=front;if direction==1,v=left;elseif direction==2,v=-left;elseif direction==3,v=-front;end
end
