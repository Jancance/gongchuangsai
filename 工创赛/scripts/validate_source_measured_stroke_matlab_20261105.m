function validate_source_measured_stroke_matlab_20261105
% 按用户尺量70+100mm重算；相关的当前伸出误差同时影响当前值及总行程。
root=fileparts(fileparts(mfilename('fullpath')));
base=fullfile(root,'evidence','integration','2026-11-05','source-markers','paper100-transfer','continuation');
out=fullfile(base,'body270x250','matlab-measured-stroke');if ~exist(out,'dir'),mkdir(out);end
ref=read_json(fullfile(root,'calibration','source_tcp_center_reference_20261105.json'));
body=read_json(fullfile(base,'body270x250','matlab-body-pose','result.json'));
old=read_json(fullfile(base,'matlab','result.json'));
assert(ref.mechanics.current_extension_ruler_measured && ref.mechanics.additional_extension_ruler_measured);
assert(ref.mechanics.source_prepare_nominal_extension_mm==70 && ref.mechanics.additional_extension_available_mm==100);
candidate=body.maximum_margin_within100mm_yaw30;
shift=candidate.chassis_right_forward_mm(:)';
q=old.offsets_right_forward_mm;
definitions={ ...
 'Measured lengths fixed; target perturbation 3mm; base exact',0,3,0; ...
 'Ruler perturbation 1mm; target 3mm; base exact',1,3,0; ...
 'Ruler perturbation 1mm; target 3mm; base 1mm',1,3,1; ...
 'Ruler perturbation 1mm; target 2mm; base 1mm',1,2,1; ...
 'Ruler perturbation 1mm; target 2mm; base 2mm',1,2,2};
scenarios=struct('label',{},'ruler_perturbation_mm',{},'target_axis_perturbation_mm',{}, ...
 'base_axis_perturbation_mm',{},'sets',{},'targets',{},'rejected_sets',{}, ...
 'minimum_extension_mm',{},'maximum_extension_mm',{},'minimum_stroke_margin_mm',{},'all_within_stroke',{});
for k=1:size(definitions,1)
 ruler=definitions{k,2};target_error=definitions{k,3};base_error=definitions{k,4};
 ruler_values=unique([-ruler,0,ruler]);target_values=unique([-target_error,0,target_error]);
 base_values=unique([-base_error,0,base_error]);
 sets=0;rejected=0;minimum=Inf;maximum=-Inf;margin=Inf;
 for variedL=[225,235,245]
  for initial_error=ruler_values
   for extra_error=ruler_values
    current=70+initial_error;maxE=current+100+extra_error;
    for dx=target_values
     for dy=target_values
      for bx=base_values
       for by=base_values
        target=[variedL+q(:,2),-q(:,1)]+[dx,dy]-(shift+[bx,by]);
        ext=hypot(target(:,1),target(:,2))-(variedL-current);
        sets=sets+1;rejected=rejected+~all(ext>=0 & ext<=maxE);
        minimum=min(minimum,min(ext));maximum=max(maximum,max(ext));
        margin=min(margin,min([ext;maxE-ext]));
       end
      end
     end
    end
   end
  end
 end
 scenarios(k)=struct('label',definitions{k,1},'ruler_perturbation_mm',ruler, ...
  'target_axis_perturbation_mm',target_error,'base_axis_perturbation_mm',base_error, ...
  'sets',sets,'targets',3*sets,'rejected_sets',rejected,'minimum_extension_mm',minimum, ...
  'maximum_extension_mm',maximum,'minimum_stroke_margin_mm',margin,'all_within_stroke',rejected==0);
end
% 可复算的不变量：当前伸出尺误差同时加到目标与最大值，远端余量不随其平移。
test_target=[235+q(:,2),-q(:,1)]-shift;
radius=hypot(test_target(:,1),test_target(:,2));
e0=radius-(235-70);e1=radius-(235-71);
checks=[max(abs((170-e0)-(171-e1)))<1e-10,scenarios(1).all_within_stroke, ...
 scenarios(4).all_within_stroke,~scenarios(3).all_within_stroke];
report=struct;report.engine='local MATLAB';report.version=version;report.release=version('-release');
report.time_asia_shanghai=char(datetime('now','TimeZone','Asia/Shanghai','Format','yyyy-MM-dd HH:mm:ss Z'));
report.reference=ref;report.candidate=candidate;report.scenarios=scenarios;
report.check_count=numel(checks);report.failure_count=sum(~checks);
report.nominal_lengths_are_ruler_measured=true;
report.correlated_stroke_formula='current=70+initial_error; maximum=current+100+extra_error; minimum_radius=L-current';
report.assumed_target_or_base_or_ruler_error_bounds_field_verified=false;
report.candidate_stroke_margin_meets_original10mm=false;
report.conditional_geometry_passed=scenarios(4).all_within_stroke;
report.full_motion_validation_passed=false;report.motion_allowed=false;report.hardware_operated=false;
report.pending={'actual ruler uncertainty','independent target/TCP error','actual base parking error', ...
 'gray lane and updated image pose','actuator dynamics/clearance/power and complete fault exits'};
write_json(fullfile(out,'result.json'),report);
save(fullfile(out,'simulation.mat'),'report','ref','q','candidate');
fprintf('MATLAB measured stroke: checks=%d failures=%d; conditional geometry=%d; motion=0\n', ...
 report.check_count,report.failure_count,report.conditional_geometry_passed);
for k=1:numel(scenarios),s=scenarios(k);fprintf('%s: sets=%d rejected=%d minmargin=%.3fmm\n',s.label,s.sets,s.rejected_sets,s.minimum_stroke_margin_mm);end
assert(all(checks),'Measured-stroke validation checks failed');
end

function value=read_json(path)
fid=fopen(path,'r','n','UTF-8');assert(fid~=-1);clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
value=jsondecode(fscanf(fid,'%c'));
end
function write_json(path,value)
fid=fopen(path,'w','n','UTF-8');assert(fid~=-1);clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(value));
end
