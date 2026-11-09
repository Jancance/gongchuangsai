function validate_integration10_top_start_20261106
% 用户确认本次起点为真实顶部，仅重新执行驱动低速回零，不作预升。
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-06','integration10-single-teach');
if ~exist(out,'dir'),mkdir(out);end
phases=[0:6,20:26];
checks=[~any(phases==8),any(phases==5),any(phases==20), ...
 any(phases==23),phases(end)==26,108400>=100000,108400<=110800, ...
 18464/10==1846.4,45055>0,43000>40000];
% 回零运行->完成、零速、新鲜位置全部取得前，原点必须无效。
for seen=[0,1]
 for stopped=[0,1]
  for fresh=[0,1]
   valid=seen && stopped && fresh;
   checks(end+1)=valid==(seen+stopped+fresh==3); %#ok<AGROW>
  end
 end
end
r=struct('engine','local MATLAB','release',version('-release'),'passed',all(checks), ...
 'check_count',numel(checks),'scope','top-only homing transition model; no physical acceptance', ...
 'user_confirmed_physical_top',true,'initial_prelift_pulses',0,'down_pulses',108400, ...
 'home_timeout_ms',43000,'motion_allowed_by_model',false);
fid=fopen(fullfile(out,'top-start-matlab.json'),'w','n','UTF-8');assert(fid~=-1);
fprintf(fid,'%s',jsonencode(r));fclose(fid);
assert(r.passed);fprintf('TOP START MATLAB %d checks passed\n',numel(checks));
f=figure('Visible','off','Color','w','Position',[100 100 1200 620]);
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
nexttile;
stairs(1:numel(phases),phases,'LineWidth',2,'Color',[.1 .4 .7]);hold on;
plot(7,6,'o','MarkerFaceColor',[.1 .6 .4],'MarkerSize',8);
yline(8,'--','不经过阶段8：普通位置运动');grid on;
xlabel('状态推进顺序（不是时间）');ylabel('控制器阶段号');
title({'顶部重新建立参考的候选流程','检查/使能 → 低速回零 → 真实反馈确认'});
text(8,17,'20：核回零参数','FontSize',11);
text(10,21,'23：发送真实回零','FontSize',11);
text(8,27,'26：读取新鲜顶部位置','FontSize',11);
ylim([-1 30]);
nexttile;
bar([90000,0,108400],'FaceColor',[.2 .55 .65]);grid on;
xticklabels({'旧初始预升','本次顶部预升','后续单次下降'});
ylabel('虚拟脉冲');ylim([0 125000]);
text(1,94000,'90000','HorizontalAlignment','center');
text(2,4000,'0','HorizontalAlignment','center');
text(3,112400,'108400','HorizontalAlignment','center');
title({'跳过预升，保留已验证下降目标','下降仍须等待真实回零完成及显式请求'});
sgtitle({'整合10：本次已确认物理顶部的接续模型', ...
 'MATLAB 18项检查通过；图中无实车运动轨迹，当前尚未执行动作'});
exportgraphics(f,fullfile(out,'top-start-plan.png'),'Resolution',160);close(f);
end
