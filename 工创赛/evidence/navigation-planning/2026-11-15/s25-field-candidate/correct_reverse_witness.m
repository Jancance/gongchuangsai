function correct_reverse_witness
out=fileparts(mfilename('fullpath'));
root=fileparts(fileparts(fileparts(fileparts(fileparts(out)))));
old=fullfile(root,'工创赛','evidence','navigation-planning','2026-11-14','baseline40-field','midsegment-1mm-feasibility');
research=fullfile(fileparts(out),'early-ff-batch');
addpath(old);addpath(research);addpath(fullfile(fileparts(out),'bounded-lateral-enhancement'));
c=load(fullfile(old,'stage_b_trace_cache.mat'),'traces');s=c.traces(1);
% Slots 1/2 are noFF/half; slots >=3 use the enhanced controller.
z=early_controller_cases(s,[1.4 .3 .2],[3 3 3 2.5],1,2,2,[],true);
time=s.t_ms/1000;along=s.along;
cross3=find(s.middle & z.right(:,3)<0,1,'first');
cross25=find(s.middle & z.right(:,4)<0,1,'first');
if isempty(cross3),at3=[];else,at3=along(cross3)/1000;end
if isempty(cross25),at25=[];else,at25=along(cross25)/1000;end
writetable(table(time,along,z.right(:,3),z.right(:,4),z.L(:,3)*.1,z.L(:,4)*.1,...
 'VariableNames',{'time_s','along_mm','S3_model_right_mm','S25_model_right_mm','S3_L_RPM','S25_L_RPM'}),fullfile(out,'corrected_reverse_witness.csv'));
summary=struct('run',s.name,'plant_effect',1.4,'tau_s',.3,'delay_s',.2,'position_bias_mm',-1,'yaw_bias_deg',-.1,...
 'cadence',2,'basis',2,'controller_scales',[3 3 3 2.5],...
 'S3_middle_peak_mm',z.peak(3),'S25_middle_peak_mm',z.peak(4),...
 'S3_middle_reverse_mm',z.reverse(3),'S25_middle_reverse_mm',z.reverse(4),...
 'S3_first_middle_crossing_m',at3,'S25_first_middle_crossing_m',at25,...
 'interpretation','Conditional fixed measured yaw and injected biases; not measured ground overshoot. Prior two-slot witness compared noFF/half, not S3/S2.5.');
f=fopen(fullfile(out,'corrected_reverse_witness.json'),'w');assert(f~=-1);fprintf(f,'%s',jsonencode(summary));fclose(f);
disp(jsonencode(summary));
end
