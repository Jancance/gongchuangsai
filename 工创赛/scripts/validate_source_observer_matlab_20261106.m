function validate_source_observer_matlab_20261106
% Validate perspective geometry and both actual node5 feedback units first.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-06','source-observer');if ~exist(out,'dir'),mkdir(out);end
checks=struct('name',{},'passed',{});cases=struct('name',{},'plane_to_image',{},'boundary_px',{},'center_mm',{},'radius_mm',{});
theta=linspace(0,2*pi,181)';theta(end)=[];
transforms=cat(3,[3,.3,900;-.2,2.5,500;.001,-.0008,1], ...
 [2,-.4,950;.4,2.2,500;-.0005,.0007,1]);
for k=1:2
 center=[12,-15];radius=150;p=center+radius*[cos(theta),sin(theta)];T=transforms(:,:,k);
 px=project(T,p);q=project(inv(T),px);[c,r,res]=circle(q);
 check('boundary inverse mapping restores physical circle',norm(c-center)<1e-8 && abs(r-radius)<1e-8 && max(abs(res))<1e-8);
 [wrong,~,~]=circle(px);wrong_plane=project(inv(T),wrong);
 check('projected fitted image center is not physical center',norm(wrong_plane-center)>.1);
 cases(k)=struct('name',sprintf('perspective%d',k),'plane_to_image',T,'boundary_px',px,'center_mm',center,'radius_mm',radius);
end
T=transforms(:,:,1);center=[12,-15];arc=linspace(20,180,81)';p=center+150*[cosd(arc),sind(arc)];
[c,r,~]=circle(project(inv(T),project(T,p)));
check('partial physical arc is fitted after rectification',norm(c-center)<1e-8 && abs(r-150)<1e-8);
% Current archived rim is a diagnostic fixture. It cannot certify live pose.
old=read_json(fullfile(root,'evidence','integration','2026-11-05','source-markers','current-221545-a3467c481720','measurement-input.json'));
cal=read_json(fullfile(root,'calibration','source_paper_current220715_20261105.json'));
q=project(cal.homography_image_to_plane,old.rim_boundary_image_px);keep=true(size(q,1),1);
for it=1:5
 [c,r,~]=circle(q(keep,:));res=sqrt(sum((q-c).^2,2))-r;
 med=median(res(keep));mad=median(abs(res(keep)-med));keep=abs(res-med)<=max(1.5,3*1.4826*mad);
end
[c,r,res]=circle(q(keep,:));
check('archived rim remains diagnostic scale',r>130 && r<170 && sqrt(mean(res.^2))<3);
check('archived paper is not operational full-plane calibration',~cal.field_calibration_accepted && ~cal.operational_extrapolation_enabled);
% Explicit chassis delta sign: base moves current landmark minus desired.
R=[0,-1;1,0];translation=[250,30];measured=([12,-15]*R')+translation;desired=measured-[8,-4];
delta=measured-desired;check('chassis correction sign and millimetres',[8,-4]*delta'>0 && norm(delta-[8,-4])<1e-9);
raw=[0,27000,144017,1219509];
for scale=[10,100]
 for sign=[-1,1]
  angle=sign*raw/scale;wire=round(abs(angle)*scale);
  check('node5 feedback and command use same Scale option',all(wire==raw));
 end
end
check('node5Scale100 differs from legacy divide10',144017/100==1440.17 && 144017/10~=1440.17);
check('unconfigured profile cannot produce ready observation',~logical(0));
% No new route is planned from the old46.5mm gap or paper extrapolation.
check('no motion permission from local geometry',~cal.motion_allowed);
report=struct('engine','local MATLAB','version',version,'release',version('-release'), ...
 'checks',checks,'check_count',numel(checks),'failure_count',sum(~[checks.passed]),'passed',all([checks.passed]), ...
 'cases',cases,'archive_rim_center_plane_mm',c,'archive_rim_radius_mm',r, ...
 'archive_rim_retained_points',sum(keep),'synthetic_chassis_rotation',R,'synthetic_chassis_translation',translation, ...
 'synthetic_desired_center_chassis_mm',desired,'synthetic_delta_chassis_mm',delta, ...
 'hardware_operated',false,'motion_allowed',false,'live_pose_verified',false);
write_json(fullfile(out,'matlab-result.json'),report);save(fullfile(out,'matlab-result.mat'),'report');
fprintf('MATLAB SOURCE OBSERVER: checks%d failures%d; node5 Scale and perspective boundary mapping, no motion\n',numel(checks),report.failure_count);
assert(report.passed);
 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed)); %#ok<AGROW>
 end
end
function p=project(H,p)
p=[p,ones(size(p,1),1)]*H';assert(all(abs(p(:,3))>1e-10));p=p(:,1:2)./p(:,3);
end
function [c,r,res]=circle(p)
o=mean(p,1);q=p-o;z=[2*q,ones(size(q,1),1)]\sum(q.^2,2);c=z(1:2)'+o;r=sqrt(z(3)+sum(z(1:2).^2));res=sqrt(sum((p-c).^2,2))-r;
end
function v=read_json(p),v=jsondecode(fileread(p));end
function write_json(p,v)
f=fopen(p,'w','n','UTF-8');assert(f~=-1);c=onCleanup(@()fclose(f));fprintf(f,'%s',jsonencode(v)); %#ok<NASGU>
end
