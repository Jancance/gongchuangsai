function matlab_ff_reportplots
out=fileparts(mfilename('fullpath'));cd(out);
fig=figure('Visible','off','Color','w','Position',[80 80 1200 500]);
for i=1:2
 data=csvread(sprintf('yaw_replay_%d.csv',i),1,0);
 subplot(1,2,i);plot(data(:,1),data(:,2),'b-',data(:,1),data(:,3),'r--','LineWidth',1.5);
 xlabel('Time (s)');ylabel('Relative heading (deg)');grid on;
 title(sprintf('Train run%d -> held-out run%d',i,3-i));
 legend('Measured HWT','Predicted from wheel models','Location','best');
 pos=get(gca,'Position');pos(2)=.16;pos(4)=.66;set(gca,'Position',pos);
end
annotation(fig,'textbox',[.04 .93 .92 .05],'String', ...
 'Model inadequacy: fitting wheel RPM alone does not validate vehicle yaw or lateral position.', ...
 'EdgeColor','none','HorizontalAlignment','center');
print(fig,'yaw_validation_failure.png','-dpng','-r150');savefig(fig,'yaw_validation_failure.fig');close(fig);
f=openfig('ff_trajectories.fig','invisible');
notes=findall(f,'Type','textboxshape');delete(notes);
annotation(f,'textbox',[.05 .955 .9 .04],'String', ...
 'Exploratory only: yaw/ground model NOT validated; backward extrapolated from forward logs.', ...
 'EdgeColor','none','HorizontalAlignment','center','FontWeight','bold');
print(f,'ff_trajectories.png','-dpng','-r140');savefig(f,'ff_trajectories.fig');close(f);
fprintf('REPORT_PLOTS_FINISHED\n');
end
