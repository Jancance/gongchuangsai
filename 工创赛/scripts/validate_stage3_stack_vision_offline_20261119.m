function validate_stage3_stack_vision_offline_20261119
% 仅验证候选数学契约；合成矩阵不是本车物料顶面标定。
cases=0;
for Jxy={ [2 .3; -.4 3], [-3 .2; .5 2], [1 0;0 -2] }
 for Jyaw=[-2 .5 2]
  for x=[-20 0 20]
   for y=[-15 0 15]
    J=Jxy{1}; residual=J*[x;y]; command=-(J\residual);
    assert(norm(residual+J*command)<1e-10);cases=cases+1;
   end
  end
  for yaw=[-89 -5 0 5 89]
   wrapped=mod(yaw+90,180)-90;
   command=-wrapped/Jyaw;
   assert(abs(wrapped+Jyaw*command)<1e-10);cases=cases+1;
  end
 end
end
% 无向直线：反向端点仍相同；yaw优先，平移使用实际当前中心。
assert(mod(179+90,180)-90==-1);
assert(mod(-179+90,180)-90==1);
fprintf('PASS stack material-plane candidate mathematics: %d cases; SYNTHETIC ONLY\n',cases+2);
end
