% Existing A2/B3 mathematics on new observation data; no control implementation.
% Run only after review. Writes only this run's stage3-resume/matlab-response.
% E: nominal outer diameter95 / center spacing150 are not field measurements.
root=fileparts(fileparts(mfilename('fullpath')));
inputPath=fullfile(root,'evidence','integration','2026-11-18','stage3-resume', ...
    'image-analysis','matlab-training-input.json');
outDir=fullfile(root,'evidence','integration','2026-11-18','stage3-resume','matlab-response');
outPath=fullfile(outDir,'response-training-frozen.json');
assert(~isfile(outPath),'Frozen training candidate already exists; do not refit/overwrite');
input=jsondecode(fileread(inputPath)); obs=input.observations;
assert(numel(obs)==4 && numel(input.pairs)==3,'Expected four poses and three training pairs');
assert(isequal({obs.id},{'P0','P1','P2','P3'}),'Expected ordered P0..P3 observations');
C=diag([10 10 3]); assert(isequal(double(input.command_rows),C),'Expected right10/forward10/turn3');
assert(input.nominal_geometry.outer_diameter_mm==95 && input.nominal_geometry.center_spacing_mm==150);
centers=double(input.reference_centers_px);
assert(isequal(size(centers),[3 2]) && all(isfinite(centers),'all'));
ellipse=input.reference_ellipse; axesFull=double(ellipse.axes_full_px(:));
assert(numel(axesFull)==2 && all(isfinite(axesFull)) && all(axesFull>0));
angle=double(ellipse.angle_deg); assert(isscalar(angle) && isfinite(angle));
rot=[cosd(angle) -sind(angle);sind(angle) cosd(angle)];
Q=rot*diag((axesFull/2).^2)*rot.';
[A2,a2RankResidual]=fitA2(centers(1,:).',centers(3,:).',Q,95/2,150);
pixelDelta=zeros(3,2); imageYawDelta=zeros(3,1); D=zeros(3,3);
imageHashes=struct('id',{},'path',{},'sha256',{});
absoluteMetricYaw=zeros(4,1);
for k=1:4
    center=double(obs(k).center_px(:)); yaw=double(obs(k).line_yaw_deg);
    assert(numel(center)==2 && all(isfinite(center)) && isscalar(yaw) && isfinite(yaw));
    actualHash=fileSha256(obs(k).image_path);
    assert(strcmpi(actualHash,obs(k).image_sha256),'Observation image SHA mismatch');
    imageHashes(k)=struct('id',obs(k).id,'path',obs(k).image_path,'sha256',actualHash);
    absoluteMetricYaw(k)=metricYaw(A2,yaw);
end
configHash=fileSha256(input.config_path);
assert(strcmpi(configHash,input.config_sha256),'Observation config SHA mismatch');
for k=1:3
    pair=input.pairs(k);
    assert(strcmp(pair.before,obs(k).id) && strcmp(pair.after,obs(k+1).id));
    assert(isequal(double(pair.command(:).'),C(k,:)) && ~pair.validation_only);
    delta=double(obs(k+1).center_px(:))-double(obs(k).center_px(:));
    pixelDelta(k,:)=delta.';
    imageYawDelta(k)=wrapLine(double(obs(k+1).line_yaw_deg)-double(obs(k).line_yaw_deg));
    % Transform both absolute line directions FIRST, then wrap their difference.
    D(k,:)=[(A2*delta).' wrapLine(absoluteMetricYaw(k+1)-absoluteMetricYaw(k))];
end
Jpix=(C\pixelDelta).'; JimageYaw=(C\imageYawDelta).';
[B,R,trainingResidual]=fitB3(C,D);
assert(all(isfinite([A2(:);Jpix(:);JimageYaw(:);R(:);B(:)])));
columnNorms=vecnorm(R,2,1); normalizedR=R./columnNorms;
command=[5;0;0]; predictedPixelDelta=Jpix*command;
predictedImageYawDelta=JimageYaw*command;
predictedImageYaw=wrapLine(double(obs(4).line_yaw_deg)+predictedImageYawDelta);
prediction=struct('from_observation','P3','command_right_forward_ccw',command.', ...
    'pixel_delta_xy',predictedPixelDelta.','image_line_yaw_delta_deg',predictedImageYawDelta, ...
    'image_center_px',(double(obs(4).center_px(:))+predictedPixelDelta).', ...
    'image_line_yaw_deg',predictedImageYaw,'linear_metric_delta_mm_mm_deg',(R*command).', ...
    'metric_yaw_delta_from_absolute_image_prediction_deg', ...
    wrapLine(metricYaw(A2,predictedImageYaw)-absoluteMetricYaw(4)), ...
    'executed',false,'independent_observation_available',false);
result=struct('status','TRAINING_CANDIDATE_ONLY_NOT_VALIDATED', ...
    'created_at',char(datetime('now','Format','yyyy-MM-dd HH:mm:ss.SSS')), ...
    'matlab_version',version,'input_path',inputPath,'input_sha256',fileSha256(inputPath), ...
    'input_metadata',input,'image_hashes',imageHashes, ...
    'reference_image_sha256',fileSha256(input.reference_image), ...
    'config_path',input.config_path,'config_sha256',configHash, ...
    'formula_source','scripts/validate_stage3_calibration_20261116.m:136-151', ...
    'nominal_outer_diameter_mm',95,'nominal_center_spacing_mm',150, ...
    'A2',A2,'A2_rank_residual',a2RankResidual,'reference_Q_px2',Q, ...
    'command_rows',C,'pixel_delta_rows',pixelDelta,'image_yaw_delta_rows_deg',imageYawDelta, ...
    'absolute_metric_yaw_deg',absoluteMetricYaw,'metric_delta_rows',D, ...
    'Jpix',Jpix,'Jimage_yaw',JimageYaw,'R',R,'B',B, ...
    'training_residual',trainingResidual,'training_residual_fro',norm(trainingResidual,'fro'), ...
    'rank_R',rank(R),'singular_values_R',svd(R),'condition_R',cond(R), ...
    'response_column_norms',columnNorms,'condition_column_normalized_R',cond(normalizedR), ...
    'rank_Jpix',rank(Jpix),'singular_values_Jpix',svd(Jpix),'condition_A2',cond(A2), ...
    'next_right5_prediction',prediction,'holdout_count',0, ...
    'calibration_verified',false,'physical_verified',false,'physical_acceptance',false, ...
    'training_residual_is_validation',false,'running_config_written',false);
result.requirement_ids_R={'RUN-01','RUN-03','FIELD-06'};
result.requirement_ids_E={'VIS-04','VIS-05','VIS-07','VIS-12','VIS-15','CAL-02','CAL-03','CAL-05','RUN-06'};
result.official_Q='Unchanged; pending technical clarification remains open';
result.limitations={'Three pairs interpolate three axes: zero training residual is not independent evidence'; ...
    'Nominal95/150 local affine approximation; no new ellipse/line/image fitting in this driver'; ...
    'Commands are commanded10mm/10mm/3deg, not independent actual travel measurements'; ...
    'Current image estimator, lens/perspective, dual-line definition and TCP remain unverified'; ...
    'Condition numbers and SVD are diagnostics only; no new stop/protection/acceptance thresholds'; ...
    'Do not write old B3true/TCP/Z or this unvalidated candidate into any running config'};
if ~isfolder(outDir), mkdir(outDir); end
% Atomic first-create reservation: an existing frozen candidate is never opened.
frozenFile=java.io.File(outPath); created=frozenFile.createNewFile();
assert(created,'Frozen training candidate already exists; no overwrite');
fid=fopen(outPath,'w','n','UTF-8'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(result)); clear cleanup;
fprintf('FROZEN TRAINING ONLY: %s; holdout=0 calibration_verified=false physical_verified=false\n',outPath);
disp('R ='); disp(R); disp('B ='); disp(B); disp('Next right5 prediction:'); disp(prediction);

function [A,residual]=fitA2(c1,c3,Q,r,s)
    vx=(c3-c1)/(2*s); q=Q/r^2-vx*vx.'; q=(q+q.')/2;
    [v,l]=eig(q); [values,order]=sort(diag(l));
    assert(values(end)>0,'Unidentifiable projection');
    vy=v(:,order(end))*sqrt(values(end)); if vy(2)<0, vy=-vy; end
    J=[vx vy]; assert(rank(J)==2,'Singular local map');
    A=inv(J); residual=abs(values(1))/values(end);
end
function y=metricYaw(A,t)
    v=A*[cosd(t);sind(t)]; y=atan2d(v(2),v(1));
end
function y=wrapLine(x), y=mod(x+90,180)-90; end
function [B,R,residual]=fitB3(C,D)
    assert(rank(C)==3,'Need independent three-axis commands');
    R=(C\D).'; assert(rank(R)==3,'Singular response');
    B=-inv(R); residual=D-C*R.';
end
function hash=fileSha256(path)
    fid=fopen(path,'rb'); assert(fid>=0,'Cannot read input evidence'); cleanup=onCleanup(@()fclose(fid));
    bytes=fread(fid,Inf,'*uint8'); clear cleanup;
    md=java.security.MessageDigest.getInstance('SHA-256'); md.update(typecast(bytes(:),'int8'));
    hash=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
