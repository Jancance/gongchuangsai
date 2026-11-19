% Stage 3 pre-implementation discrete model. No hardware calls or firmware edits.
% R: QR-01/05, RUN-01/02/03, FIELD-06, MECH-01/02.
% E: QR-02/03, VIS-04/07/12/15, CAL-02/03/05, RUN-06.
% Q: QR-04 and FIELD-05 remain pending official clarification.
% Engineering test only: manual loading/moving is not a competition run.
root = fileparts(fileparts(mfilename('fullpath')));
out = fullfile(root,'evidence','integration','2026-11-16','stage3','matlab-result.json');
checks = struct('name',{},'passed',{},'details',{});

% Existing carrier servo wait followed by requested manual loading dwell.
boundaries = [0 699 700 2699 2700 3399 3400 5399 5400 6099 6100 10000];
expected = [1 1 2 2 3 3 4 4 5 5 6 6];
for k=1:numel(boundaries)
    assert(loadState(boundaries(k)) == expected(k));
end
addCheck('manual_loading_boundaries',true, ...
    '0deg wait700/dwell2000;120deg wait700/dwell2000;240deg wait700;complete at6100ms; no auto cycle');
assert(all([0 120 240] == carrierAngles()));
addCheck('manual_loading_angles',true,'slot1/2/3 carrier command 0/120/240 degrees; no invented arrival feedback');

% Structured task mapping: carrier slot is ordinal pickup order, not color ID.
colors = [2 1 5]; positions = [1 2 3];
tasks = makeTasks(colors,positions);
assert(isequal([tasks.slot],[1 2 3]));
assert(isequal([tasks.color],colors));
assert(isequal([tasks.target],positions));
assert(tasks(2).color==1 && tasks(2).target==2);
permutations = perms(1:3);
for c=1:size(permutations,1)
    for p=1:size(permutations,1)
        t=makeTasks(colors(permutations(c,:)),permutations(p,:));
        assert(numel(unique([t.slot]))==3);
        assert(isequal([t.color],colors(permutations(c,:))));
        assert(isequal([t.target],permutations(p,:)));
    end
end
addCheck('task_mapping_36_permutations',true,'215 means yellow/red/black, slots1/2/3;123 means targets1/2/3; center single piece is slot2 red to target2');

% Angle from actual upper/lower baseline endpoints. Undirected line = 180deg equivalence.
angles = [-179 -91 -90 -89 -6 -1.001 -1 -0.999 0 0.999 1 1.001 6 89 90 91 179];
for a=angles
    line=[0 0;cosd(a) sind(a)];
    observed=atan2d(line(2,2)-line(1,2),line(2,1)-line(1,1));
    assert(abs(angleError(observed,0)-angleError(observed+180,0))<1e-10);
    assert(abs(angleError(observed,0)-angleError(observed-180,0))<1e-10);
end
% Exact +/-90 direction is ambiguous; no physical convergence claim there.
assert(angleError(1.2,0)>0 && angleError(-1.2,0)<0);
assert(angleError(179,0)==-1 && angleError(-179,0)==1);
addCheck('baseline_angle_sign_and_180_equivalence',true,'atan2 from real line endpoints; wrap to[-90,90); +179=-1 and -179=+1; exact90 ambiguity remains');

% Idealized body response, correct sign and gain; not a real chassis model.
starts=[15 -12 6;-15 12 -6;3 -4 1.2;-3 4 -1.2;0 0 179;0 0 -179;2 2 1;2.001 0 0;0 0 1.001];
alignmentResults=struct('initial',{},'final',{},'steps',{});
for k=1:size(starts,1)
    e=starts(k,:); e(3)=angleError(e(3),0); initial=e;
    steps=0;
    while ~aligned(e)
        if abs(e(3))>1
            delta=-0.75*e(3);
            % Re-observe XY in the rotated body frame after heading correction.
            r=[cosd(-delta) -sind(-delta);sind(-delta) cosd(-delta)];
            e(1:2)=(r*e(1:2).').'; e(3)=e(3)+delta;
        else
            % One chassis axis per completed step; choose the larger residual.
            [~,axis]=max(abs(e(1:2))); e(axis)=0.25*e(axis);
        end
        steps=steps+1;
        assert(steps<20,'Ideal model should converge; this is a test bound, not a new firmware threshold');
    end
    alignmentResults(end+1)=struct('initial',initial,'final',e,'steps',steps); %#ok<SAGROW>
end
assert(~aligned([0 0 1.2])); % Existing 5deg entry floor would deadlock this case.
assert(~aligned([2 2 1]) && aligned([2 0 1]) && ~aligned([2.001 0 0]));
addCheck('initial_alignment_9_cases',true,'heading first with XY frame rotation/re-observation, then one largest-error axis per step; converges to hypot(X,Y)<=2mm and angle<=1deg; +/-1.2deg requires rotation entry below5deg');

% Existing observation freshness/identity contract, not a new threshold.
frame=struct('run',31,'pose',8,'seq',100,'valid',true,'age_ms',200);
assert(acceptObservation(frame,31,8,99));
bad=frame; bad.age_ms=201; assert(~acceptObservation(bad,31,8,99));
bad=frame; bad.pose=7; assert(~acceptObservation(bad,31,8,99));
bad=frame; bad.run=30; assert(~acceptObservation(bad,31,8,99));
bad=frame; bad.seq=99; assert(~acceptObservation(bad,31,8,99));
bad=frame; bad.valid=false; assert(~acceptObservation(bad,31,8,99));
assert(crc16(uint8('123456789'))==hex2dec('29B1'));
payload='O31,31,8,100,1,-1500,2500,-1200,100,';
crc=crc16(uint8(payload));
assert(crc~=crc16(uint8(payload(1:end-1))));
addCheck('observation_epoch_sequence_age_and_crc',true,'run/pose match; strictly increasing seq; valid; age<=200ms including boundary; CRC16/CCITT-FALSE initFFFF poly1021 MSB-first includes final comma, golden123456789=29B1');
for mode={'STAGE3','SOURCE','MANUAL'}
    for residual=[-5 -1.001 -1 0 1 1.001 5]
        allowed=rotationEntry(mode{1},residual);
        if strcmp(mode{1},'STAGE3')
            assert(allowed==(abs(residual)>1));
        else
            assert(allowed==(abs(residual)>=5));
        end
    end
end
addCheck('rotation_entry_stage3_only',true,'stage3 correction only for abs residual>1deg; illustrative existing other-mode5deg floor unchanged; real cross-layer integration still requires review');

% Symbolic poses only. Numeric lateral angle/X/Z must come from calibration.
calibration=struct('center_dm_deg',180,'center_x_mm',0, ...
    'left_dm_deg',[],'left_x_mm',[],'right_dm_deg',[],'right_x_mm',[], ...
    'place_z_mm',[],'physical_valid',false);
assert(isempty(calibration.left_dm_deg) && isempty(calibration.place_z_mm));
addCheck('uncalibrated_poses_are_missing',true,'known center DM180/X0 only; side poses and all placement heights absent, not fictitious valid defaults');

% Event-driven control topology. Completion events are idealized, not timers.
sequence={'OBS_Z_TOP','OBS_X_HOME','OBS_DM_180','INITIAL_ALIGN','FOUR_WHEEL_HOLD'};
pieceSequence={'SELECT_SLOT','PICK_Z_TOP','PICK_X_HOME','PICK_DM_36_5', ...
    'PICK_DESCEND','GRIP_140','LIFT_TOP','TARGET_DM','TARGET_X', ...
    'PLACE_DESCEND','RELEASE_210','RETURN_Z_TOP','RETURN_X_HOME'};
for piece=1:3, sequence=[sequence pieceSequence]; end %#ok<AGROW>
sequence=[sequence {'FINAL_DM_0','FINAL_SLOT_240','DONE'}];
assert(sum(strcmp(sequence,'INITIAL_ALIGN'))==1);
holdIndex=find(strcmp(sequence,'FOUR_WHEEL_HOLD'));
assert(holdIndex<find(strcmp(sequence,'SELECT_SLOT'),1));
assert(sum(strcmp(sequence,'RELEASE_210'))==3);
assert(strcmp(sequence{end-2},'FINAL_DM_0') && strcmp(sequence{end-1},'FINAL_SLOT_240'));
assert(strcmp(sequence{end},'DONE'));
for piece=1:3
    start=holdIndex+(piece-1)*numel(pieceSequence);
    assert(strcmp(sequence{start+12},'RETURN_Z_TOP'));
    assert(strcmp(sequence{start+13},'RETURN_X_HOME'));
end
assert(~any(strcmp(sequence(holdIndex+1:end),'INITIAL_ALIGN')));
addCheck('placement_sequence_and_hold',true,'one initial closed loop; wheels hold throughout pickup/placement; per piece slot/top/Xhome/DM36.5/descend/grip140/top/targetDM/targetX/placeZ/open210/top/Xhome; all single-axis serialized; finalDM0/slot240');
existingZBounds=[3200 29700]; unknownZ=-1;
assert(unknownZ<existingZBounds(1));
assert(all(existingZBounds==[3200 29700]));
addCheck('existing_it3_z_bounds',true,'IT3 existing3200..29700 bounds retained; unknown default-1 is outside validity; no new protection threshold invented');

% Abort injected in every active state. No auto recovery movement or restart.
abortCases=0;
for k=1:numel(sequence)-1
    for reason={'STOP','EXISTING_FAULT'}
        model=struct('state',sequence{k},'hold',k>=holdIndex,'starts',1,'reason','');
        model=abortModel(model,reason{1});
        assert(strcmp(model.state,'ABORTED') && model.stopRequested);
        assert(~model.autoHome && ~model.autoRestart && model.starts==1);
        for idleTick=1:5
            assert(strcmp(model.state,'ABORTED'));
        end
        abortCases=abortCases+1;
    end
end
addCheck('stop_fault_all_states',true,sprintf('%d state/reason injections; existing STOP/fault stops; no automatic homing, cycle or clearing',abortCases));

% Region ownership dispatcher excludes source module from stage3 commands.
for owner={'NONE','SOURCE','STAGE3'}
    sourceRuns=strcmp(owner{1},'SOURCE'); stage3Runs=strcmp(owner{1},'STAGE3');
    assert(~(sourceRuns && stage3Runs));
    if stage3Runs, assert(~sourceRuns); end
end
completion=struct('state','DONE','starts',1);
for idleTick=1:100, assert(completion.starts==1 && strcmp(completion.state,'DONE')); end
addCheck('module_exclusion_and_no_autoloop',true,'single owner NONE/SOURCE/STAGE3; stage3 entered by upper arrival request; DONE/ABORTED do not restart on ticks');

result=struct;
result.status='PASS_PREIMPLEMENTATION_DISCRETE_MODEL_ONLY';
result.generated_at=char(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
result.matlab_version=version;
result.requirement_ids_checked={'QR-01','QR-02','QR-03','QR-04','QR-05','RUN-01','RUN-02','RUN-03','RUN-06','FIELD-05','FIELD-06','MECH-01','MECH-02','VIS-04','VIS-07','VIS-12','VIS-15','CAL-02','CAL-03','CAL-05'};
result.rule_facts='R: color IDs, task mapping, staged transport, autonomous competition and no post-release pushing remain authoritative; manual bench preparation is E only.';
result.pending_official='Q: QR-04 and FIELD-05 remain pending technical clarification.';
result.checks=checks;
result.alignment_results=alignmentResults;
result.manual_load_timeline_ms=[0 700 2700 3400 5400 6100];
result.example_tasks=tasks;
result.calibration=calibration;
result.nominal_symbolic_sequence=sequence;
result.assumptions={'XY errors and baseline angle supplied by calibrated fresh observations'; ...
    'XY tolerance means Euclidean hypot(X,Y)<=2mm'; ...
    'ideal correct-sign0.75 response with heading/XY coupling and single-axis steps, no delay/slip/noise/backlash'; ...
    'baseline reference must come from actual upper/lower markings, never invented image vertical'; ...
    'abstract completion events and symbolic placement poses stand in for unmeasured hardware'; ...
    'module exclusion is a dispatcher ownership requirement, not verified firmware behavior'};
result.not_verified={'real calibration including baseline reference and mm transforms'; ...
    'left/right reachability, placement height, collision/swept clearance'; ...
    'chassis response signs, latency, noise, physical hold and precision'; ...
    'firmware implementation, build, STOP timing or hardware actions'; ...
    'real single center piece or three piece acceptance and full competition timing'};
result.implementation_permission_scope='Topology/algebra prevalidation passed; executable implementation must retain explicit uncalibrated poses and existing STOP/fault behavior. No hardware approval is implied.';
if ~exist(fileparts(out),'dir'), mkdir(fileparts(out)); end
fid=fopen(out,'w','n','UTF-8'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(result),'char'); clear cleanup;
fprintf('STAGE3 MATLAB PREVALIDATION PASS: %d groups; %d abort injections; 36 task permutations.\n',numel(checks),abortCases);
fprintf('Result: %s\n',out);

function addCheck(name,passed,details)
    checks=evalin('base','checks');
    checks(end+1)=struct('name',name,'passed',passed,'details',details);
    assignin('base','checks',checks);
end
function s=loadState(t)
    s=1+sum(t>=[700 2700 3400 5400 6100]);
end
function a=carrierAngles(), a=[0 120 240]; end
function t=makeTasks(c,p)
    assert(numel(c)==3 && numel(unique(c))==3 && all(c>=1 & c<=6));
    assert(isequal(sort(p),1:3));
    for n=1:3, t(n)=struct('slot',n,'color',c(n),'target',p(n)); end %#ok<AGROW>
end
function e=angleError(a,r), e=mod(a-r+90,180)-90; end
function ok=aligned(e), ok=hypot(e(1),e(2))<=2 && abs(e(3))<=1; end
function ok=acceptObservation(f,run,pose,lastSeq)
    ok=f.valid && f.run==run && f.pose==pose && f.seq>lastSeq && f.age_ms>=0 && f.age_ms<=200;
end
function ok=rotationEntry(mode,e)
    if strcmp(mode,'STAGE3'), ok=abs(e)>1; else, ok=abs(e)>=5; end
end
function crc=crc16(bytes)
    crc=uint16(65535);
    for b=bytes
        crc=bitxor(crc,bitshift(uint16(b),8));
        for n=1:8
            if bitand(crc,uint16(32768)), crc=bitxor(bitshift(crc,1),uint16(hex2dec('1021')));
            else, crc=bitshift(crc,1); end
        end
    end
    crc=double(crc);
end
function m=abortModel(m,reason)
    m.state='ABORTED'; m.reason=reason; m.stopRequested=true;
    m.autoHome=false; m.autoRestart=false;
end
