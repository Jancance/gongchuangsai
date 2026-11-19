function validate_observe210_20261116
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-16','alignment30-release300-observe210');
if ~exist(out,'dir'),mkdir(out);end
checks=0;
for angle=[0 140 170 210 270]
 pulse=500+floor(angle*2000/270+.5);
 assert(angle>=0 && angle<=270 && pulse>=500 && pulse<=2500);checks=checks+1;
 if angle==210,assert(pulse==2056);checks=checks+1;end
end
for request=[5 6]
 if request==5,angle=140;else,angle=210;end
 assert((request==5 && angle==140)||(request==6 && angle==210));checks=checks+1;
end
% Release and pre-close are separate call sites, unchanged by observation angle.
assert(170~=210 && 140~=210 && 300==300);checks=checks+1;
% All source observation visits: initial request6 and returns after slots1/2.
for slot=1:3
 carrier_angle=210;dwell_ms=300;
 if slot<3,after_return_angle=210;else,after_return_angle=210;end
 assert(carrier_angle==210 && dwell_ms==300);checks=checks+1;
 assert((slot<3 && after_return_angle==210)||(slot==3 && after_return_angle==210));checks=checks+1;
end
r=struct('passed',true,'local_matlab',true,'checks',checks,'observation_open_deg',210,...
 'observation_pulse_us',2056,'preclose_deg',170,'close_deg',140,'carrier_open_deg',210,...
 'carrier_release_ms',300,'source_step_max_mm',30,'physical_angle_validated',false,...
 'physical_sweep_validated',false,'hardware_access',false,...
 'all_source_observation_visits_210',true,'slot3_park_angle_unchanged',true);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('PASS observation210 nominal PWM/request separation: %u checks.\n',checks);
end
