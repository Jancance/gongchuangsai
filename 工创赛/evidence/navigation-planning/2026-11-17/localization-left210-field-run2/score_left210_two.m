function score_left210_two
out=fileparts(mfilename('fullpath'));base=fileparts(out);diary(fullfile(out,'score_matlab_diary.log'));summary=cell(1,2);ios=cell(1,2);
for k=1:2
 folder=fullfile(base,sprintf('localization-left210-field-run%d',k));s=jsondecode(fileread(fullfile(folder,'holdout_prediction.json')));m=jsondecode(fileread(fullfile(folder,'operator_measurement.json')));q=readtable(fullfile(folder,'holdout_decomposition.csv'));d=jsondecode(fileread(fullfile(folder,'capture','decoded.json')));tr=double(d.header.trigger_ms);
 summary{k}=struct('run',k,'ground_left_mm',m.actual_left_mm,'ground_forward_mm',m.forward_mm,'model_left_mm',s.main_left_mm,'model_forward_mm',s.cross_forward_mm,'main_apparent_error_mm',s.main_left_mm-m.actual_left_mm,'baseline_cross_apparent_error_mm',s.cross_forward_mm-m.forward_mm,'fixed_cross_prediction_mm',s.fixed_corrected_cross_prediction_mm,'fixed_cross_apparent_error_mm',s.fixed_corrected_cross_prediction_mm-m.forward_mm,'ratio_cross_prediction_mm',s.proportional_corrected_cross_prediction_mm,'ratio_cross_apparent_error_mm',s.proportional_corrected_cross_prediction_mm-m.forward_mm,'common_end_s',s.common_end_s,'terminal_gap_s',s.terminal_minus_common_end_s,'training_unchanged',true,'synchronized_ground_truth',false);
 files=dir(fullfile(folder,'serial','*.raw'));assert(numel(files)==1);txt=fileread(fullfile(files(1).folder,files(1).name));lines=regexp(txt,'IO1,[^\r\n]*','match');v=[];bad=0;
 for j=1:numel(lines)
  line=lines{j};sep=find(line==',',1,'last');if isempty(sep),bad=bad+1;continue;end
  f=strsplit(line,',');if numel(f)~=29,bad=bad+1;continue;end
  if crc16(line(1:sep))~=hex2dec(f{end}),bad=bad+1;continue;end
  values=str2double(f(2:28));assert(numel(values)==27 && all(isfinite(values)));v(end+1,:)=values;
 end
 assert(bad==0);valid=v(:,6)==1 & v(:,7)==0;source=(v(:,9)-tr)/1000;valid_support=valid & source>=q.source_s(1) & source<=q.source_s(end);z=find(valid_support);compare=cell(1,numel(z));
 for j=1:numel(z)
  n=z(j);wheel=interp1(q.source_s,[q.path_F_mm q.path_L_mm],source(n));icm=[v(n,21) -v(n,22)]/1000;
  compare{j}=struct('tx_tick_ms',v(n,2),'estimate_ms',v(n,9),'source_s',source(n),'reasons',v(n,7),'flags',v(n,8),'ICM_F_mm',icm(1),'ICM_L_mm',icm(2),'wheel_F_mm',wheel(1),'wheel_L_mm',wheel(2),'ICM_minus_wheel_F_mm',icm(1)-wheel(1),'ICM_minus_wheel_L_mm',icm(2)-wheel(2),'cal_samples',v(n,16),'gaps',v(n,18));
 end
 compare=[compare{:}];assert(~isempty(compare));writetable(struct2table(compare),fullfile(folder,'IO1_valid_same_source.csv'));
 reason_codes=unique(v(:,7))';counts=zeros(size(reason_codes));for j=1:numel(reason_codes),counts(j)=sum(v(:,7)==reason_codes(j));end
 ios{k}=struct('run',k,'legal_IO1_frames',size(v,1),'bad_crc_or_fields',bad,'valid_frames',sum(valid),'valid_in_wheel_support',numel(z),'reason_codes',reason_codes,'reason_counts',counts,'cal_samples_min',min(v(:,16)),'cal_samples_max',max(v(:,16)),'max_reported_gaps',max(v(:,18)),'first_valid_source_s',min(source(valid)),'last_valid_source_s',max(source(valid)),'last_valid_supported_source_s',compare(end).source_s,'last_supported_ICM_F_mm',compare(end).ICM_F_mm,'last_supported_ICM_L_mm',compare(end).ICM_L_mm,'last_supported_difference_F_mm',compare(end).ICM_minus_wheel_F_mm,'last_supported_difference_L_mm',compare(end).ICM_minus_wheel_L_mm,'max_abs_difference_F_mm',max(abs([compare.ICM_minus_wheel_F_mm])),'max_abs_difference_L_mm',max(abs([compare.ICM_minus_wheel_L_mm])),'final_frame_valid',logical(valid(end)),'final_frame_reasons',v(end,7),'final_frame_source_s',source(end),'invalid_cache_excluded',true,'position_R_converted_to_L',true,'shared_wheel_not_independent_ground_truth',true);
end
summary=[summary{:}];ios=[ios{:}];writetable(struct2table(summary),fullfile(out,'two_run_holdout_scores.csv'));S=struct('status','PASS_DIAGNOSTIC_WITH_LIMITATIONS','matlab_version',version,'runs',summary,'IO1',ios,'training_refit',false,'no_strictly_synchronized_ground',true,'baseline_MAE_mm',mean(abs([summary.baseline_cross_apparent_error_mm])),'fixed_MAE_mm',mean(abs([summary.fixed_cross_apparent_error_mm])),'ratio_MAE_mm',mean(abs([summary.ratio_cross_apparent_error_mm])));fid=fopen(fullfile(out,'two_run_holdout_scores.json'),'w','n','UTF-8');fwrite(fid,jsonencode(S),'char');fclose(fid);disp(S);diary off;
end
function crc=crc16(str)
crc=uint16(65535);for byte=uint8(str),crc=bitxor(crc,bitshift(uint16(byte),8));for b=1:8,if bitand(crc,uint16(32768)),crc=bitxor(bitshift(crc,1),uint16(4129));else,crc=bitshift(crc,1);end,end,end
crc=double(crc);
end
