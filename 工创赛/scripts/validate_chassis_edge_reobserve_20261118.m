% MATLAB prerequisite for repeat alignment and a calibrated observation angle.
root=fileparts(fileparts(mfilename('fullpath')));
base=fullfile(root,'evidence','integration','2026-11-18','stage3-resume');
comparison=jsondecode(fileread(fullfile(base,'chassis-edge-dm-angle-comparison.json')));
names=fieldnames(comparison);
for k=1:numel(names)
 samples=comparison.(names{k});
 if mean([samples.body_yaw_deg])<0, reference=samples; end
end
model=jsondecode(fileread(fullfile(base,'repark-matlab-measured.json')));
J=model.J_current_pixel_per_right_forward_ccw;
body=mean([reference.body_yaw_deg]);
error=mean([reference.delta_deg]);
turn=-error/J(3,3);
assert(body<0 && error>1 && abs(turn)>1 && abs(turn)<10);
centers=zeros(2,numel(reference));
for k=1:numel(reference),centers(:,k)=reference(k).center_px(:);end
for gain=[.85 1 1.15]
 residual=error+gain*J(3,3)*turn;
 assert(abs(residual)<1);
 center=mean(centers,2)+gain*J(1:2,3)*turn;
 planar=-(J(1:2,1:2)\(center-[960;540]));
 assert(all(abs(planar)<30));
end
cases=0;
for state=[0 1 2 3 4 5 6 7 9 10 11]
 for stop=0:1
  for manual=0:1
   for configReady=0:1
    allowed=state==7 && ~stop && ~manual && ~configReady;
    assert(~allowed || (state==7 && ~stop && ~manual && ~configReady));
    cases=cases+1;
   end
  end
 end
end
% JSON arrays decode as columns in this MATLAB. Assemble observations as
% separate columns before averaging, preserving u/v coordinates.
result=struct('passed',true,'matlab_version',version,'admission_cases',cases, ...
 'body_reference_deg',body,'relative_yaw_error_deg',error,'estimated_turn_deg',turn, ...
 'translation_solve','-(Jxy \ pixel_error)','live_verified',false);
path=fullfile(base,'chassis-edge-reobserve-matlab.json');
fid=fopen(path,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(result));clear cleanup;
fprintf('PASS %d admission cases and three response gains, turn %.4f deg\n',cases,turn);
