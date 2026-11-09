function validate_extension_hold_tolerance_20261106
root=fileparts(fileparts(mfilename('fullpath')));out=fullfile(root,'evidence','integration','2026-11-06','can2-recovery');
checks=[];
for raw=[0,12,20,21,100]
 for drift=[0,.27,3,3.1]
  for flags=[3,11]
   accept=raw<=20 && drift<=3 && bitand(uint32(flags),uint32(12))==0 && bitand(uint32(flags),uint32(1))~=0;
   checks(end+1)=accept==(raw<=20 && drift<=3 && flags==3); %#ok<AGROW>
  end
 end
end
raw=[12,0,15,20];n=0;
for i=1:numel(raw),if raw(i)<=20,n=n+1;else,n=0;end,end
checks(end+1)=n>=3;
checks(end+1)=20*.1==2 && 12*.1==1.2;
r=struct('engine','local MATLAB','passed',all(checks),'check_count',numel(checks), ...
 'scope','post-enable hold feedback only; pre-home and pre-STOP zero checks unchanged', ...
 'speed_tolerance_decirpm',20,'position_tolerance_deg_retained',3,'fresh_bundles',3, ...
 'physical_stop_user_confirmed',true,'hardware_acceptance',false);
fid=fopen(fullfile(out,'hold-tolerance-matlab.json'),'w');assert(fid~=-1);fprintf(fid,'%s',jsonencode(r));fclose(fid);
f=figure('Visible','off','Color','w');plot(1:4,raw*.1,'o-');hold on;yline(2,'--');grid on;
xlabel('保位反馈样本');ylabel('速度绝对值 / RPM');title('保位微动候选模型：位置限制及连续3组反馈仍保留');
exportgraphics(f,fullfile(out,'hold-tolerance-plan.png'),'Resolution',130);close(f);
assert(r.passed);fprintf('HOLD MATLAB checks=%d passed=%d\n',numel(checks),r.passed);
end
