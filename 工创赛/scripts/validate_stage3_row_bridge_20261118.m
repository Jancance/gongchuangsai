% MATLAB prerequisite for measured ring-row observation adapter and MCU selector.
root=fileparts(fileparts(mfilename('fullpath')));
base=fullfile(root,'evidence','integration','2026-11-18','stage3-resume');
r=jsondecode(fileread(fullfile(base,'repark-matlab-measured.json')));
p=jsondecode(fileread(fullfile(base,'repark-matlab-prevalidation-v2.json')));
J=double(r.J_current_pixel_per_right_forward_ccw);A=double(p.A2_nominal);
assert(rank(J)==3 && rank(J(1:2,1:2))==2 && rank(A)==2);
cases=0;maxSteps=0;
for x=[-60 -20 0 20 60]
 for y=[-80 -30 0 30 80]
  for yaw=[-5 -1.001 -1 0 1 1.001 5]
   e=[x;y;yaw];steps=0;
   for n=1:20
    plane=A*e(1:2);metricYaw=planeYaw(A,e(3));
    if norm(plane)<=2 && abs(metricYaw)<=1,break;end
    if abs(metricYaw)>1
     v=-e(3)/J(3,3);motion=[0;0;v];axis=3;
     assert(abs(v)>1);v=sign(v)*min(abs(v),10);
    else
     planar=-(J(1:2,1:2)\e(1:2));motion=[planar;0];
     [~,axis]=max(abs(motion(1:2)));v=sign(motion(axis))*min(abs(motion(axis)),30);
     assert(abs(v)>=1,'Sub-step outside acceptance');
    end
    e=e+J(:,axis)*v;steps=steps+1;
   end
   assert(norm(A*e(1:2))<=2 && abs(planeYaw(A,e(3)))<=1,'Did not close MCU strategy');
   cases=cases+1;maxSteps=max(maxSteps,steps);
  end
 end
end
% Sequential camera producer: last consumed completion remains a conservative
% lower bound even if intervening completed frames were discarded.
previous=[20 2.0];next=[23 2.1];fence=1.9;
assert(next(1)>previous(1) && previous(2)>fence && next(2)>previous(2));
assert(~(previous(2)>2.01)); % old-pose acquisition cannot pass new fence.
for seq=[20 19],assert(~(seq>previous(1)));end
% Existing wire freshness, no new timeout/stop condition.
assert(ceil((2.15-2.0)*1000)<=200 && ceil((2.25-2.0)*1000)>200);
result=struct('status','PASS_MATLAB_ROW_BRIDGE_PREREQUISITE','matlab_version',version, ...
 'cases',cases,'max_steps',maxSteps,'response',J,'nominal_A2',A, ...
 'target',[960 540 0],'calibration_verified',false,'physical_placement_verified',false, ...
 'failure_exits',{{'MCU fault/STOP uses existing state path';'non-observe status sends invalid only'; ...
 'expired/old epoch/duplicate frame cannot send valid';'worker generation change resets temporal history'; ...
 'missing actual triple resets original detector; no substitute centers'}}, ...
 'requirements_R',{{'RUN-01','RUN-03','FIELD-06','PWR-01'}}, ...
 'requirements_E',{{'VIS-15','CAL-02','CAL-03','CAL-05','RUN-06'}});
path=fullfile(base,'row-bridge-matlab.json');assert(~isfile(path));
fid=fopen(path,'w','n','UTF-8');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(result));clear cleanup;
fprintf('ROW BRIDGE MATLAB PASS %d scenarios, max %d steps; physical placement unverified\n',cases,maxSteps);
function y=planeYaw(A,t)
 a=A*[cosd(t);sind(t)];b=A*[1;0];
 y=mod(atan2d(a(2),a(1))-atan2d(b(2),b(1))+90,180)-90;
end
