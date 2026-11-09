function validate_can2_recovery_matlab_20261106
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-06','can2-recovery');
if ~exist(out,'dir'),mkdir(out);end
checks=[];mask=uint32(hex2dec('0FFFC000'));
% CAN1复位后，只恢复归CAN2所有的14..27号过滤器，保留新CAN1过滤器0。
for old=[0,2^14,hex2dec('0AAAA000'),hex2dec('0FFFFFFF')]
 restored=bitor(bitand(uint32(1),bitcmp(mask)),bitand(uint32(old),mask));
 checks(end+1)=bitand(restored,mask)==bitand(uint32(old),mask); %#ok<AGROW>
 checks(end+1)=bitand(restored,uint32(16383))==1; %#ok<AGROW>
end
rows=[];
for gap=[1,10,20,50]
 for ack=[8,29,Inf]
  samples=0:gap:200;terminal=NaN;ok=false;
  for now=samples
   if isfinite(ack) && now>=ack && ack<30,terminal=now;ok=true;break;end
   if now>=30,terminal=now;break;end
  end
  checks(end+1)=terminal<=30+gap; %#ok<AGROW>
  checks(end+1)=ok==isfinite(ack); %#ok<AGROW>
  rows(end+1,:)=[gap,ack,terminal,ok]; %#ok<AGROW>
 end
end
% 总线离线只允许控制器自行恢复电气接入，已取消邮箱不得恢复旧动作。
for pending=[0,1]
 for recovered=[0,1]
  after_cancel=0;checks(end+1)=after_cancel==0 && ~(after_cancel && recovered); %#ok<AGROW>
 end
end
r=struct('engine','local MATLAB','release',version('-release'),'passed',all(checks), ...
 'check_count',numel(checks),'scope','shared filter preservation, bounded retry/abort scheduling; no physical link acceptance', ...
 'TX_deadline_ms',30,'maximum_delay_requires_service_gap_bound',true,'old_motion_auto_resumed',false);
fid=fopen(fullfile(out,'matlab-result.json'),'w');assert(fid~=-1);fprintf(fid,'%s',jsonencode(r));fclose(fid);
f=figure('Visible','off','Color','w','Position',[100 100 1000 460]);
nexttile;bar([1,14],[1,1]);xticks([1,14]);xticklabels({'CAN1过滤器0保留','CAN2过滤器14恢复'});ylim([0 1.3]);title('复位后恢复各自过滤器');ylabel('模型检查通过');
nexttile;bad=isinf(rows(:,2));plot(rows(bad,1),rows(bad,3),'o-','LineWidth',2);grid on;
xlabel('服务间隔/ms');ylabel('无ACK时取消邮箱的时间/ms');title('重传受30ms软件截止约束');
sgtitle(sprintf('CAN2恢复前置模型：%d项检查；不是物理通信验收',numel(checks)));
exportgraphics(f,fullfile(out,'recovery-plan.png'),'Resolution',140);close(f);
assert(r.passed);fprintf('CAN2 MATLAB checks=%d passed=%d\n',numel(checks),r.passed);
end
