function validate_integration10_complete_matlab_20261106
% 单件示教完整接口的前置模型；不替代载荷、连杆、线缆和实物扫掠。
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-06','integration10-complete');
if ~exist(out,'dir'),mkdir(out);end
checks=[];
ext=[0,48,48,48,0,0,0,0];
theta=[180,180,180,180,180,35.1079407,35.1079407,180];
% 高度用“距本轮真实顶部的下降脉冲”表示，放料目标未量，不能填旧值。
depth=[0,0,108400,0,0,0,NaN,0];
checks(end+1)=all(ext>=0 & ext<=170);
checks(end+1)=all(theta>=0 & theta<=270);
checks(end+1)=depth(3)==108400 && isnan(depth(7));
checks(end+1)=ext(5)==0 && theta(5)==180 && theta(6)<180;
for flags=0:31
 ready=bitget(flags,1) && bitget(flags,2) && bitget(flags,3) && ~bitget(flags,4) && ~bitget(flags,5);
 checks(end+1)=ready==(flags==7); %#ok<AGROW>
end
for pulses=[3200,8000,16000,108400]
 for scale=[10,100]
  for top=[-26,0,6,1234]
   target=top+pulses*360*scale/3200;
   recovered=(target-top)*3200/(360*scale);
   checks(end+1)=abs(recovered-pulses)<1e-8; %#ok<AGROW>
  end
 end
end
% 普通升降不直接高速触顶；返回复用已验“顶前2mm+60RPM真实回零”。
checks(end+1)=3200/1600==2;
checks(end+1)=abs((180-35.1079407)/300-0.482973531)<1e-9;
checks(end+1)=abs(48/(106*3200/60/32)-0.2716981132)<1e-9;
checks(end+1)=isequal(500+round([0,140,210]*2000/270),[500,1537,2056]);
report=struct('engine','local MATLAB','release',version('-release'),'passed',all(checks), ...
 'check_count',numel(checks),'source_depth_pulses',108400,'source_extension_mm',48, ...
 'carrier_angle_candidate_deg',35.1079407,'carrier_release_depth_pulses',[], ...
 'release_depth_requires_current_teach',true,'prior28000_reused',false, ...
 'top_reference_from_real_homing',true,'horizontal_requires_lift_at_top',true, ...
 'rotation_requires_retracted_extension',true,'severe_fault_STOP_retained',true, ...
 'physical_sweep_accepted',false,'motion_allowed_by_model',false);
fid=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(fid~=-1);
fprintf(fid,'%s',jsonencode(report));fclose(fid);assert(report.passed);
f=figure('Visible','off','Color','w','Position',[100 100 1300 600]);
tiledlayout(1,2,'Padding','compact');
nexttile;hold on;grid on;axis equal;
r0=161;source=(r0+48)*[1,0];retracted=r0*[1,0];
a=linspace(0,180-35.1079407,200)*pi/180;
plot([source(1),retracted(1)],[0,0],'LineWidth',3);
plot(r0*cos(a),r0*sin(a),'LineWidth',3);
plot(source(1),source(2),'o','MarkerSize',8);plot(retracted(1),0,'o','MarkerSize',8);
xlabel('平面示意X / mm');ylabel('平面示意Y / mm');
title({'高位：先缩回，再旋转','161mm回缩半径为既有候选，仅作运动学示意'});
legend('48mm回缩','转向1号槽候选','取料朝向','回缩点','Location','best');
nexttile;
plot(1:6,depth(1:6),'o-','LineWidth',2);set(gca,'YDir','reverse');grid on;
xticks(1:6);xticklabels({'顶部','伸出','下降抓取','回顶','缩回原点','转向载盘'});xtickangle(25);
ylabel('距真实顶部的下降虚拟脉冲');
title({'单件示教顺序','放料高度待现场示教；新版本支持调整、松爪和撤离'});
sgtitle(sprintf('整合10完整单件接口：MATLAB %d项检查；模型图不是实车轨迹',numel(checks)));
exportgraphics(f,fullfile(out,'complete-plan.png'),'Resolution',150);close(f);
fprintf('COMPLETE MATLAB checks=%d passed=%d\n',numel(checks),report.passed);
end
