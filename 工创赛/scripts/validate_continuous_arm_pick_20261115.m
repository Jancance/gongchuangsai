function validate_continuous_arm_pick_20261115
% Prior to firmware: same-start continuous descent and full loaded retraction.
% Motor turns derived from the actual VSP degree conversion, NOT old 10x X model.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-15','continuous-arm-pick');
if ~exist(out,'dir'),mkdir(out);end
ppmX=315.5;ppmZ=400;unitsPerRev=3200;
xMax=150;zMax=1846.4;aX=600/60;aZ=45055/60;
pick=29700;hover=15600;top=600;place=1800;
yaw=[195 165 180];extension=[164.07083627876932 161.96991633111764 0];
centers=[(161+extension).*cosd(yaw-180);(161+extension).*sind(yaw-180)]';
budget=40+25+4;checks=0;rows={};count=0;minimum=Inf;
% Existing 90ms start-skew diagnostic as a declared timing assumption, not
% a new runtime abort threshold. X may start after Z by up to this amount.
skew=.09;
for region=1:3
 for initialX=[60 65 70]
  for initialZ=[400 600 800]
   count=count+1;
   dx=abs(extension(region)-initialX)*ppmX/unitsPerRev;
   dz=(pick-initialZ)/unitsPerRev;dh=(hover-initialZ)/unitsPerRev;
   tx=duration(dx,xMax/60,aX);
   speed=selectspeed(dz,dh,tx+skew,zMax,aZ);
   tz=duration(dz,speed/60,aZ);th=crossing(dh,dz,speed/60,aZ);
   assert(th>=tx+skew-1e-10 && speed>0 && speed<=zMax);checks=checks+2;
   assert(tz<12 && tx<12);checks=checks+2;
   % One nonzero-velocity descent. No intermediate target, zero-speed dwell,
   % or second CAN position request at the old hover height.
   time=linspace(0,max(tx+skew,tz),1501);
   [z,zv]=profile(time,dz,speed/60,aZ);[x,~]=profile(max(time-skew,0),dx,xMax/60,aX);
   depth=initialZ+z*unitsPerRev;
   posX=initialX+sign(extension(region)-initialX)*x*unitsPerRev/ppmX;
   assert(all(diff(depth)>=-1e-8));checks=checks+1;
   inside=time>0 & time<tz;assert(all(zv(inside)>0));checks=checks+1;
   low=depth>=hover;assert(all(abs(posX(low)-extension(region))<1e-5));checks=checks+1;
   % Return: both axes start at t0, full X0 target immediately. Loaded yaw
   % stays at pickup angle until BOTH finish and the existing >65mm rise.
   rx=extension(region)*ppmX/unitsPerRev;rz=(pick-place)/unitsPerRev;
   trX=duration(rx,xMax/60,aX);trZ=duration(rz,zMax/60,aZ);
   rt=linspace(0,max(trX,trZ)+skew,1801);
   [xm,~]=profile(rt,rx,xMax/60,aX);
   % Worst ordering for collisions: X starts first; Z starts 90ms later.
   [zm,~]=profile(max(rt-skew,0),rz,zMax/60,aZ);
   rise=zm*unitsPerRev/ppmZ;retracted=xm*unitsPerRev/ppmX;
   assert(abs(retracted(end)-extension(region))<1e-8 && abs(rise(end)-(pick-place)/ppmZ)<1e-8);checks=checks+2;
   for n=find(rise<=65)
    point=(161+extension(region)-retracted(n))*[cosd(yaw(region)-180) sind(yaw(region)-180)];
    for other=setdiff(1:3,region)
     gap=norm(point-centers(other,:));minimum=min(minimum,gap);
     assert(gap>budget);checks=checks+1;
    end
   end
   assert(rise(end)>65 && max(trX,trZ)+skew<12);checks=checks+2;
   rows{count}=struct('region',region-1,'initial_extension_mm',initialX,...
    'initial_lift_pulses',initialZ,'target_extension_mm',extension(region),...
    'lift_rpm',speed,'extension_duration_s',tx,'hover_crossing_s',th,...
    'contact_duration_s',tz,'return_extension_duration_s',trX,...
    'return_lift_duration_s',trZ); %#ok<AGROW>
  end
 end
end
% Deterministic comparison vectors for C, including triangular short moves,
% unchanged maximum-speed paths and encoder-scale values at travel endpoints.
vectors={};n=0;
for initialX=[0 60 65 70 170]
 for goalX=[0 65 162 170]
   for initialZ=[0 600 800]
   n=n+1;dx=abs(goalX-initialX)*ppmX/unitsPerRev;
   dz=(pick-initialZ)/unitsPerRev;dh=(hover-initialZ)/unitsPerRev;
   speed=selectspeed(dz,dh,duration(dx,xMax/60,aX)+skew,zMax,aZ);
   vectors{n}=struct('initial_x',initialX,'goal_x',goalX,'initial_z',initialZ,'lift_rpm',speed); %#ok<AGROW>
  end
 end
end
% Existing STOP/fault exits: analytically bound ideal deceleration at every
% sampled path phase. This does NOT establish actual drive STOP latency.
for rpm=[xMax zMax]
 a=aX;if rpm==zMax,a=aZ;end
 for fraction=[0 .25 .5 .75 1]
  v=rpm/60*fraction;t=v/a;distance=v*t-.5*a*t*t;
  assert(t>=0 && distance>=0 && abs(v-a*t)<1e-10);checks=checks+3;
 end
end
r=struct('passed',true,'local_matlab',true,'checks',checks,'scenarios',count,...
 'rows',[rows{:}],'comparison_vectors',[vectors{:}],'extension_rpm',xMax,'lift_max_rpm',zMax,...
 'extension_accel_rpm_s',600,'lift_accel_rpm_s',45055,...
 'assumed_start_skew_ms',90,'minimum_loaded_neighbour_center_distance_mm',minimum,...
 'declared_tool_radius_mm',40,'declared_material_radius_mm',25,'existing_model_budget_mm',4,...
 'actual_3d_tool_carrier_cable_sweep_measured',false,'actual_drive_stop_latency_verified',false,...
 'continuous_descent_single_target',true,'loaded_full_retract_same_start',true,...
 'contact_prediction_timing_requires_recalibration',true,'hardware_access',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('PASS continuous arm MATLAB: %u checks/%u scenarios; loaded min gap %.2fmm; actual VSP units, no hardware\n',checks,count,minimum);
end

function rpm=selectspeed(d,h,required,maxrpm,a)
% Quantise DOWN to the protocol's0.1RPM so rounding cannot make Z early.
lo=0;hi=floor(maxrpm*10);
while lo<hi
 mid=ceil((lo+hi)/2);
 if crossing(h,d,mid/600,a)>=required,lo=mid;else,hi=mid-1;end
end
assert(lo>0);rpm=lo/10;
end
function t=duration(d,v,a)
if d==0,t=0;elseif d<=v*v/a,t=2*sqrt(d/a);else,t=d/v+v/a;end
end
function t=crossing(s,d,v,a)
peak=min(v,sqrt(d*a));ta=peak/a;da=.5*a*ta*ta;total=duration(d,v,a);
if s<=da,t=sqrt(2*s/a);elseif s<d-da,t=ta+(s-da)/peak;else,t=total-sqrt(2*(d-s)/a);end
end
function [s,v]=profile(t,d,limit,a)
if d==0,s=zeros(size(t));v=s;return;end
peak=min(limit,sqrt(d*a));ta=peak/a;total=duration(d,limit,a);
s=zeros(size(t));v=s;
for k=1:numel(t)
 x=t(k);
 if x>=total,s(k)=d;
 elseif x<ta,s(k)=.5*a*x*x;v(k)=a*x;
 elseif x<=total-ta,s(k)=.5*a*ta*ta+peak*(x-ta);v(k)=peak;
 else,s(k)=d-.5*a*(total-x)^2;v(k)=a*(total-x);end
end
end
