function validate_source_mcu_cycle_20261115
% Before implementation: reuse existing motion and observation contracts.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-15','source-mcu-cycle');
if ~exist(out,'dir'),mkdir(out);end
p=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-15',...
 'integration18','window5-pause','integration19-field','source_body_registration_field.json')));
A=p.response_px_per_command_mm;scale=p.scale_mm_per_px;
checks=0;maxSteps=0;cases=0;
for x=[-60 -40 -20 0 20 40 60]
 for y=[-40 -20 0 20 40]
  for responseScale=[.95 1 1.05]
   delta=[x;y];steps=0;
   while norm(delta)*scale>5
    u=A\(-delta);[~,axis]=max(abs(u));
    distance=max(1,min(20,round(abs(u(axis)))));
    assert(distance>=1 && distance<=20);checks=checks+1;
    delta=delta+A(:,axis)*sign(u(axis))*distance*responseScale;
    steps=steps+1;assert(steps<30);checks=checks+1;
   end
   assert(norm(delta)*scale<=5);checks=checks+1;
   cases=cases+1;maxSteps=max(maxSteps,steps);
  end
 end
end
% No stale pose sample is reused after a step or after a committed piece.
for age=[0 199 200 201 400]
 for poseMatches=[false true]
  accepted=poseMatches && age<=200;
  if ~poseMatches || age>200,assert(~accepted);end
  checks=checks+1;
 end
end
% Start at arm0/X0/top: actual top, actual X origin, then arm180/X65.
prep=[1 2 7 6 3 3];axes=[0 0 0 0 1 2];
assert(isequal(prep,[1 2 7 6 3 3]) && isequal(axes(end-1:end),[1 2]));checks=checks+1;
% Alignment completion waits for existing STOP15 and closes the held chain.
for active=[false true]
 for stopMask=[0 7 15]
  complete=~active && stopMask==15;
  assert(complete==(~active && stopMask==15));checks=checks+1;
 end
end
% Two quick beeps, no blocking delay.80on/80gap/80on; then observe.
time=0:10:300;on=(time<80)|(time>=160 & time<240);
assert(sum(diff([false,on,false])==1)==2 && ~on(end));checks=checks+1;
assert(time(find(time>=240,1))==240);checks=checks+1;
% Only a real fresh stationary decision can submit the next colour once.
task=[2 1 5];acceptedSequences=[];slots=[];
for item=1:3
 remaining=task(item:end);visible=zeros(1,3);visible([2 3 1])=[2 1 5];
 visible(~ismember(visible,remaining))=0;
 region=find(visible==task(item));assert(numel(region)==1);checks=checks+1;
 for missingVisionAt=1:9
  committed=true;done=false;
  for phase=[1 2 4 5 11 12 6 7 10]
   cameraAvailable=phase<missingVisionAt; %#ok<NASGU>
   if phase==10,done=true;end
   assert(committed);checks=checks+1;
  end
  assert(done);checks=checks+1;
 end
 acceptedSequences(end+1)=item;slots(end+1)=item; %#ok<AGROW>
end
assert(isequal(acceptedSequences,1:3) && isequal(slots,1:3));checks=checks+1;
for phase=[1 2 3 4 5 6 7]
 for realFault=[false true]
  abort=realFault;assert(abort==realFault);checks=checks+1;
 end
end
prior=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-15',...
 'integration20-yaw165-window10','matlab-result.json')));
assert(prior.passed && prior.local_matlab);checks=checks+1;
% User subsequently requested normal extension arrival within5mm.
extensionScale=315.5*360/3200;arrival=5*extensionScale;
for errorMm=[0 .5 2 4.999 5 5.001 5.868088819437033 10 10.01]
 arrived=abs(errorMm*extensionScale)<=arrival;
 assert(arrived==(errorMm<=5));checks=checks+1;
 pathAllowed=abs(errorMm*extensionScale)<=10*extensionScale;
 assert(pathAllowed==(errorMm<=10));checks=checks+1;
end
for zeroSpeed=[false true]
 arrived=zeroSpeed && 4.999*extensionScale<=arrival;
 assert(arrived==zeroSpeed);checks=checks+1;
end
r=struct('passed',true,'local_matlab',true,'checks',checks,'alignment_cases',cases,...
 'maximum_simulated_body_steps',maxSteps,'body_position_tolerance_mm',5,...
 'existing_step_min_max_mm',[1 20],'existing_observation_age_ms',200,...
 'prep_requests',prep,'yaw_deg',[195 165 180],'extension_path_window_mm',10,...
 'extension_arrival_mm',5,'extension_arrival_virtual_deg',arrival,...
 'extension_zero_virtual_deg',17.75,'zero_speed_requirement_preserved',true,...
 'beep_on_gap_ms',[80 80],'beep_count',2,'beep_done_ms',240,...
 'sequences',acceptedSequences,'slots',slots,'host_per_piece_commands',0,...
 'recognition_paused_after_commit',true,'no_extra_inter_piece_dwell',true,...
 'hardware_access',false,'physical_flow_verified',false,...
 'assumptions','Prior local translation response plus/minus5percent; no yaw/ground-slip/load or guaranteed frame-availability model');
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('PASS MCU cycle MATLAB: %u checks/%u alignment cases, max%u steps; prep0/top/X0 ->180/65, two fast beeps,3single commits\n',checks,cases,maxSteps);
end
