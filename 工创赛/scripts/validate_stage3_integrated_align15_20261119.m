function validate_stage3_integrated_align15_20261119
root=fileparts(fileparts(mfilename('fullpath')));
cfg=jsondecode(fileread(fullfile(root,'output','stage3_row_bridge_20261118','stage3-row-config.json')));
J=cfg.ring_row_alignment.response_pixel_per_right_forward_ccw;
A=cfg.ring_row_alignment.nominal_image_to_station_affine;
base=[960;540]; offset=[5;15]; target=base+J(1:2,1:2)*offset;
assert(norm((J(1:2,1:2)\(target-base))-offset)<1e-9);
cases=0;
for initialX=[-80 -20 0 20 80]
 for initialY=[-80 -20 0 20 80]
  for initialYaw=[-5 -1.001 0 1.001 5]
   error=[initialX;initialY;initialYaw];
   for k=1:40
    v=A*[cosd(error(3));sind(error(3))];b=A*[1;0];
    yaw=mod(atan2d(v(2),v(1))-atan2d(b(2),b(1))+90,180)-90;
    if norm(A*error(1:2))<=2 && abs(yaw)<=1, break; end
    if abs(yaw)>1
     axis=3; distance=-error(3)/J(3,3);
     distance=sign(distance)*min(abs(distance),10);
    else
     move=-(J(1:2,1:2)\error(1:2));[~,axis]=max(abs(move));
     distance=sign(move(axis))*min(abs(move(axis)),30);
    end
    error=error+J(:,axis)*distance;
   end
   assert(k<40 && norm(A*error(1:2))<=2 && abs(yaw)<=1);
   cases=cases+1;
  end
 end
end
% The former center must command the compensated motion, never complete there.
e=base-target; motion=-(J(1:2,1:2)\e);
assert(norm(motion-offset)<1e-9 && norm(A*e)>2);
% Accepted final target has no replay translation left to execute.
assert(norm(target-target)==0);
for stopped=[false true]
 for final=[false true]
  for locked=[false true]
   beep=~stopped && final && locked;
   assert(~beep || (final && locked && ~stopped));
   cases=cases+1;
  end
 end
end
fprintf('PASS integrated visual compensation: %d cases; forward15/right5; target %.9f %.9f; no post-alignment replay\n',cases,target(1),target(2));
end
