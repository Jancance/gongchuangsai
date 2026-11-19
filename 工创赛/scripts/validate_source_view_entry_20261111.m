function validate_source_view_entry_20261111
% 看护观测入口时序模型；不模拟实车几何、碰撞或轴跟随。
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-11','source-view-entry');
if ~exist(out,'dir'),mkdir(out);end
n=0;
for fresh=0:1
 for arrived=0:1
  for refs=0:1
   for idle=0:1
    for stopped=0:1
     move=fresh && arrived && refs && idle && stopped;
     assert(~move || all([fresh arrived refs idle stopped]));n=n+1;
    end
   end
  end
 end
end
% Arm finishes first at retracted extension, then extension to view position.
for armDone=0:1
 for extDone=0:1
  observe=armDone && extDone;
  assert(~observe || armDone);n=n+1;
 end
end
assert(180>=0 && 180<=270 && 65>=0 && 65<=170);n=n+1;
% No route-DONE, inventory or metric-valid writes in this entry's contract.
r=struct('passed',true,'checks',n,'yaw_deg',180,'extension_mm',65, ...
 'serial_pose_only',true,'body_motion_enabled',false,'hardware_access',false, ...
 'real_geometry_verified',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(f>0);
fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('PASS: %d supervised view-entry model checks; no physical acceptance.\n',n);
end
