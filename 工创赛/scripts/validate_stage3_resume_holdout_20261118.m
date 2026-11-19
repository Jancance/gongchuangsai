% Independent right5 evaluation only; never fit or update the frozen candidate.
% Input: image-analysis/right5-holdout-input.json, schema_version=1:
% command_right_forward_ccw:[5,0,0], validation_only:true,
% frozen_sha256, config_path, config_sha256, same_original_algorithm:true,
% estimator_note: original CLI/outer middle ring + dual-line yaw provenance,
% before/after:{image_path (absolute),image_sha256,center_px:[x,y],
% line_yaw_deg,mcu_pose,source_cli_json_path,source_cli_json_sha256}.
% before MUST be frozen P3/turn3-after-20261118-061601.png, after new right5 raw.
% mcu_command_json_path (absolute),mcu_command_json_sha256 records original
% stage3-live command evidence, whose completed/after fault/state/pose/seq are read.
% E nominal geometry only; physical/calibration verified remain false.
root=fileparts(fileparts(mfilename('fullpath')));
base=fullfile(root,'evidence','integration','2026-11-18','stage3-resume');
frozenPath=fullfile(base,'matlab-response','response-training-frozen.json');
inputPath=fullfile(base,'image-analysis','right5-holdout-input.json');
frozenHash=fileSha256(frozenPath);
assert(strcmp(frozenHash,'20c3f0e9139551024896571f37af5aa047a05b35d6613b0ed2f04b23c9a03996'),'Frozen candidate drift');
f=jsondecode(fileread(frozenPath));i=jsondecode(fileread(inputPath));
assert(i.schema_version==1 && i.validation_only && i.same_original_algorithm);
assert(strcmpi(i.frozen_sha256,frozenHash) && strlength(string(i.estimator_note))>0);
assert(isequal(double(i.command_right_forward_ccw(:)),[5;0;0]));
assert(strcmpi(i.config_sha256,f.config_sha256));
assert(strcmpi(fileSha256(i.config_path),f.config_sha256));
assert(strcmpi(fileSha256(f.input_path),f.input_sha256),'Training metadata drift');
obs=[i.before i.after];p3=f.input_metadata.observations(4);
assert(strcmpi(obs(1).image_sha256,p3.image_sha256) && strcmp(obs(1).image_path,p3.image_path),'Before must be frozen P3');
assert(isequal(double(obs(1).center_px(:)),double(p3.center_px(:))) && obs(1).line_yaw_deg==p3.line_yaw_deg);
assert(~any(strcmpi(obs(2).image_sha256,{f.image_hashes.sha256})),'After must be independent of training images');
absoluteMetricYaw=zeros(2,1);A2=double(f.A2);R=double(f.R);
for k=1:2
    assert(isAbsolute(obs(k).image_path) && isAbsolute(obs(k).source_cli_json_path));
    assert(strcmpi(fileSha256(obs(k).image_path),obs(k).image_sha256),'Image SHA mismatch');
    assert(strcmpi(fileSha256(obs(k).source_cli_json_path),obs(k).source_cli_json_sha256),'CLI evidence SHA mismatch');
    c=double(obs(k).center_px(:));t=double(obs(k).line_yaw_deg);
    assert(numel(c)==2 && all(isfinite(c)) && isscalar(t) && isfinite(t));
    absoluteMetricYaw(k)=metricYaw(A2,t);
end
assert(isAbsolute(i.mcu_command_json_path));
assert(strcmpi(fileSha256(i.mcu_command_json_path),i.mcu_command_json_sha256),'MCU evidence SHA mismatch');
mcu=jsondecode(fileread(i.mcu_command_json_path));
assert(mcu.completed && isempty(mcu.first_error) && ~mcu.reset && ~mcu.flashed);
assert(strcmp(mcu.action,'body_axis') && numel(mcu.writes)==3,'Expected original single body-axis command evidence');
assert(isequal({mcu.writes.symbol},{'g_source_align_direction','g_source_align_distance_mm','g_stage3_request'}));
assert(isequal(double([mcu.writes.index]),[0 0 0]) && isequal(double([mcu.writes.value]),[2 1084227584 20]),'Expected right5 float32 command, not measured travel');
assert(mcu.before.g_stage3_pose==obs(1).mcu_pose && mcu.after.g_stage3_pose==obs(2).mcu_pose);
assert(mcu.after.g_stage3_state==2 && mcu.after.g_stage3_fault==0 && mcu.after.g_stage3_config_ready==0);
assert(mcu.after.g_source_align_command_seq==mcu.before.g_source_align_command_seq+1);
command=[5;0;0];prediction=f.next_right5_prediction;
assert(isequal(double(prediction.command_right_forward_ccw(:)),command));
pixelDelta=double(obs(2).center_px(:))-double(obs(1).center_px(:));
yawDelta=wrapLine(double(obs(2).line_yaw_deg)-double(obs(1).line_yaw_deg));
pixelResidual=pixelDelta-double(prediction.pixel_delta_xy(:));
yawResidual=wrapLine(yawDelta-double(prediction.image_line_yaw_delta_deg));
% Transform absolute directions separately BEFORE wrapping their difference.
measuredMetricDelta=[A2*pixelDelta;wrapLine(absoluteMetricYaw(2)-absoluteMetricYaw(1))];
predictedMetricDelta=R*command;metricResidual=measuredMetricDelta-predictedMetricDelta;
metricResidual(3)=wrapLine(metricResidual(3));
commandHat=R\measuredMetricDelta;commandError=commandHat-command;
result=struct('status','INDEPENDENT_HOLDOUT_DIAGNOSTICS_ONLY', ...
    'created_at',char(datetime('now','Format','yyyy-MM-dd HH:mm:ss.SSS')), ...
    'matlab_version',version,'input_path',inputPath,'input_sha256',fileSha256(inputPath), ...
    'input_metadata',i,'frozen_path',frozenPath,'frozen_sha256_before',frozenHash, ...
    'command_right_forward_ccw',command.','pixel_delta_xy',pixelDelta.', ...
    'image_line_yaw_delta_deg',yawDelta,'pixel_prediction_residual_xy',pixelResidual.', ...
    'pixel_prediction_residual_norm_px',norm(pixelResidual), ...
    'image_yaw_prediction_residual_deg',yawResidual,'absolute_nominal_metric_yaw_deg',absoluteMetricYaw.', ...
    'measured_nominal_metric_delta_mm_mm_deg',measuredMetricDelta.', ...
    'predicted_nominal_metric_delta_mm_mm_deg',predictedMetricDelta.', ...
    'nominal_metric_prediction_residual_mm_mm_deg',metricResidual.', ...
    'nominal_metric_translation_residual_norm_mm',norm(metricResidual(1:2)), ...
    'command_hat_right_forward_ccw',commandHat.','command_hat_error_mm_mm_deg',commandError.', ...
    'command_hat_translation_error_norm_mm',norm(commandError(1:2)), ...
    'command_hat_yaw_error_deg',commandError(3),'condition_R',cond(R), ...
    'condition_column_normalized_R',cond(R./vecnorm(R,2,1)),'condition_A2',cond(A2), ...
    'mcu_completed',mcu.completed,'mcu_after_fault',mcu.after.g_stage3_fault, ...
    'mcu_pose_before',mcu.before.g_stage3_pose,'mcu_pose_after',mcu.after.g_stage3_pose, ...
    'mcu_command_seq_before',mcu.before.g_source_align_command_seq, ...
    'mcu_command_seq_after',mcu.after.g_source_align_command_seq, ...
    'holdout_count',1,'candidate_refitted',false,'running_config_written',false, ...
    'calibration_verified',false,'physical_verified',false,'physical_acceptance',false);
result.requirement_ids_R={'RUN-01','RUN-03','FIELD-06'};
result.requirement_ids_E={'VIS-04','VIS-05','VIS-07','VIS-12','VIS-15','CAL-02','CAL-03','CAL-05','RUN-06'};
result.official_Q='Unchanged; pending technical clarification remains open';
result.limitations={'Nominal metric values are model coordinates, not independently measured physical travel'; ...
    'One right-axis holdout does not validate forward/yaw axes, TCP, lens or full operating region'; ...
    'Estimator provenance is supplied by main/reviewer; this driver checks evidence hashes, not image algorithms'; ...
    'No acceptance thresholds, control writes, new stop/protection conditions or automatic verified=true'; ...
    'Low residuals and zero training residual do not constitute physical calibration acceptance'};
result.frozen_sha256_after=fileSha256(frozenPath);
assert(strcmp(result.frozen_sha256_after,frozenHash),'Frozen candidate changed during evaluation');
outDir=fullfile(base,'matlab-holdout');if ~isfolder(outDir),mkdir(outDir);end
outPath=fullfile(outDir,'right5-holdout-evaluation.json');
newFile=java.io.File(outPath);assert(newFile.createNewFile(),'Evaluation already exists; no overwrite/repeated evaluation');
fid=fopen(outPath,'w','n','UTF-8');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(result));clear cleanup;
fprintf('Independent holdout diagnostics: %s; calibration_verified=false physical_verified=false\n',outPath);
disp(result);

function y=metricYaw(A,t)
    v=A*[cosd(t);sind(t)]; y=atan2d(v(2),v(1));
end
function y=wrapLine(x), y=mod(x+90,180)-90; end
function yes=isAbsolute(path)
    yes=~isempty(regexp(path,'^([A-Za-z]:[\\/]|/)','once'));
end
function hash=fileSha256(path)
    fid=fopen(path,'rb');assert(fid>=0,'Cannot read input evidence');cleanup=onCleanup(@()fclose(fid));
    bytes=fread(fid,Inf,'*uint8');clear cleanup;
    md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(bytes(:),'int8'));
    hash=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
