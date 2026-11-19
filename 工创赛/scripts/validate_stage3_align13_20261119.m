function validate_stage3_align13_20261119
root=fileparts(fileparts(mfilename('fullpath')));
cfg=jsondecode(fileread(fullfile(root,'output','stage3_row_bridge_20261118','stage3-row-config.json')));
J=cfg.ring_row_alignment.response_pixel_per_right_forward_ccw;
A=cfg.ring_row_alignment.nominal_image_to_station_affine;
count=0; targets=zeros(2,3);
for right=[0 5]
 offset=[right;13]; target=[960;540]+J(1:2,1:2)*offset;
 assert(norm(J(1:2,1:2)\(target-[960;540])-offset)<1e-9);
 targets(1+(right==5),:)=[right,target'];
 for x=[-80 -20 0 20 80]
  for y=[-80 -20 0 20 80]
   for angle=[-5 -1.001 0 1.001 5]
    e=[x;y;angle];
    for k=1:40
     v=A*[cosd(e(3));sind(e(3))]; b=A*[1;0];
     yaw=mod(atan2d(v(2),v(1))-atan2d(b(2),b(1))+90,180)-90;
     if norm(A*e(1:2))<=2 && abs(yaw)<=1, break; end
     if abs(yaw)>1
      axis=3; distance=-e(3)/J(3,3); limit=10;
     else
      move=-(J(1:2,1:2)\e(1:2)); [~,axis]=max(abs(move));
      distance=move(axis); limit=30;
     end
     e=e+J(:,axis)*(sign(distance)*min(abs(distance),limit));
    end
    assert(k<40 && norm(A*e(1:2))<=2 && abs(yaw)<=1);
    count=count+1;
   end
  end
 end
 % Center alone is not the final target; no separate offset is appended.
 assert(norm(A*([960;540]-target))>2);
 assert(norm(-(J(1:2,1:2)\([960;540]-target))-offset)<1e-9);
end
fprintf('PASS align13: %d ideal-response convergence cases; right0 target %.12f %.12f; right5 target %.12f %.12f\n',count,targets(1,2:3),targets(2,2:3));
fprintf('Model only: measured response assumed constant; no hardware or ground accuracy acceptance.\n');
end
