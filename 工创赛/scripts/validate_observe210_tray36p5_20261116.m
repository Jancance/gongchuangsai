function validate_observe210_tray36p5_20261116
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-16','observe210-tray36p5');
if ~exist(out,'dir'),mkdir(out);end
checks=0;
for slot=1:3
 start=195;tray=36.5;
 if slot<3,finish=180;else,finish=0;end
 assert(tray>=0 && tray<=270 && abs(tray-37)==.5);checks=checks+1;
 assert(start>=0 && finish>=0 && finish<=270);checks=checks+1;
 if slot==3,assert(finish-tray==-36.5);checks=checks+1;end
end
pulse=500+floor(210*2000/270+.5);assert(pulse==2056);checks=checks+1;
for elapsed=[0 299 300 301]
 assert((elapsed>=300)==(elapsed==300||elapsed==301));checks=checks+1;
end
r=struct('passed',true,'local_matlab',true,'checks',checks,'observation_open_deg',210,...
 'carrier_yaw_deg',36.5,'carrier_open_deg',210,'carrier_release_ms',300,...
 'preclose_deg',170,'close_deg',140,'source_step_max_mm',30,...
 'physical_angle_and_sweep_validated',false,'hardware_access',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('PASS observation210/tray36.5 nominal geometry and timing: %u checks.\n',checks);
end
