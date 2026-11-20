function chassis_baseline40_feasibility_20261113
% Engineering feasibility only: no motor dynamics, slip, hardware or deployment.
root = fileparts(fileparts(mfilename('fullpath')));
repo = fileparts(root);
outDir = fullfile(root,'evidence','navigation-planning','2026-11-13','baseline40-matlab');
if ~exist(outDir,'dir'), mkdir(outDir); end
% R2016b on Windows reads M files in the system code page; keep source ASCII.
projectDir = fullfile(repo,['3.stm32f407' char([24037 31243])]);
sourceDir = fullfile(projectDir,'CHASSIS','USER');
app = fileread(fullfile(sourceDir,'app_config.h'));
route = fileread(fullfile(sourceDir,'motion_route_trial.h'));
kinematics = fileread(fullfile(sourceDir,'chassis_kinematics.h'));
vehicle = fileread(fullfile(projectDir,'USER','vehicle_parameters.h'));
wheelDiameter = macroNumber(vehicle,'VEHICLE_WHEEL_DIAMETER_MM');
distanceScale = macroNumber(kinematics,'CHASSIS_FORWARD_DISTANCE_SCALE');
followRpm = macroNumber(route,'ROUTE_TRIAL_FOLLOW_RPM');
omega = macroNumber(app,'APP_DISTANCE_SCURVE_FOLLOW_OMEGA');
commandGuard = macroNumber(route,'ROUTE_TRIAL_MAX_RPM');
feedbackGuard = macroNumber(route,'ROUTE_TRIAL_FEEDBACK_RPM');
finalWaitMs = macroNumber(app,'APP_DISTANCE_SCURVE_FINAL_WAIT_MS');
marginMs = macroNumber(route,'ROUTE_TRIAL_MARGIN_MS');
assert(wheelDiameter==77 && distanceScale==1.02182540 && followRpm==2 && omega==1);
factor = pi*wheelDiameter/60*distanceScale;
accelReserve = factor*2*followRpm*omega;
jerkReserve = factor*6*followRpm*omega^2;
distance = 2100; speed = 400; dt = 0.002;
cases = [macroNumber(app,'APP_DISTANCE_SCURVE_ACCEL_MM_S2'), ...
    macroNumber(app,'APP_DISTANCE_SCURVE_JERK_MM_S3');150 300;200 400;300 600];
rows = zeros(size(cases,1),19);
for n = 1:size(cases,1)
    accel = cases(n,1)-accelReserve; jerk = cases(n,2)-jerkReserve;
    assert(accel>0 && jerk>0,'Follow reserve must leave positive A/J.');
    [phase,duration,peak,endpointError] = prepareProfile(distance,[speed accel jerk]);
    % Include every exact phase boundary as well as the 2 ms samples.
    t = unique([(0:dt:duration)';phase(:,1);duration]);
    [position,velocity,acceleration,jerks] = sampleProfile(phase,t,distance,duration);
    cruise = phase(5,1)-phase(4,1);
    timeoutMs = ceil(duration*1000)+finalWaitMs+marginMs;
    assert(endpointError<=0.002+distance*1e-5);
    assert(abs(position(1))<1e-9 && abs(position(end)-distance)<1e-9);
    assert(all(diff(position)>=-1e-7),'Position must be monotonic.');
    assert(abs(velocity(1))<1e-9 && abs(velocity(end))<1e-9);
    assert(abs(acceleration(1))<1e-9 && abs(acceleration(end))<1e-9);
    assert(max(velocity)<=speed+1e-8 && min(velocity)>=-1e-8);
    assert(max(abs(acceleration))<=accel+1e-8 && max(abs(jerks))<=jerk+1e-8);
    assert(duration*1000<56000 && timeoutMs<=60000);
    assert(duration*1000+finalWaitMs<timeoutMs);
    reached = abs(peak-speed)<1e-8 && cruise>0;
    rpm = peak/factor; commandRpm = ceil(rpm)+followRpm;
    rows(n,:) = [cases(n,:) accel jerk duration peak cruise endpointError ...
        max(abs(acceleration)) max(abs(jerks)) rpm commandRpm timeoutMs ...
        reached reached&&cruise>=1 rpm>feedbackGuard commandRpm>commandGuard ...
        commandRpm-commandGuard rpm-feedbackGuard];
    fprintf('CASE A=%.0f J=%.0f effective_A=%.9f effective_J=%.9f peak=%.9f duration=%.9f cruise400=%.9f rpm=%.9f command_budget=%.0f timeout_ms=%.0f\n', ...
        cases(n,1),cases(n,2),accel,jerk,peak,duration,cruise,rpm,commandRpm,timeoutMs);
end
assert(rows(1,14)==0 && rows(1,6)<speed && rows(1,7)==0,'Original 40/80 must remain the unreachable control.');
assert(all(rows(2:end,14)==1),'Every candidate must contain a real 400 mm/s cruise.');
eligible = find(rows(:,15)==1);
assert(~isempty(eligible),'No candidate has the engineering observation plateau.');
selected = eligible(1); % Candidates ordered by increasing A and J.
names = {'requested_A_mm_s2','requested_J_mm_s3','effective_A_mm_s2','effective_J_mm_s3', ...
    'duration_s','peak_speed_mm_s','cruise400_s','integrated_endpoint_error_mm', ...
    'sampled_peak_A_mm_s2','sampled_peak_J_mm_s3','peak_wheel_rpm','command_rpm_budget', ...
    'timeout_ms','reaches400_with_cruise','cruise_at_least1s','feedback80_conflict', ...
    'command52_conflict','command_guard_excess_rpm','feedback_guard_excess_rpm'};
resultTable = array2table(rows,'VariableNames',names);
writetable(resultTable,fullfile(outDir,'profile_cases.csv'));
report = struct('matlab_version',version,'distance_mm',distance,'speed_limit_mm_s',speed, ...
    'wheel_diameter_mm',wheelDiameter,'forward_distance_scale',distanceScale, ...
    'wheel_mm_s_per_rpm',factor,'follow_rpm',followRpm,'follow_omega',omega, ...
    'acceleration_reserve_mm_s2',accelReserve,'jerk_reserve_mm_s3',jerkReserve, ...
    'sample_dt_s',dt,'cases',table2struct(resultTable),'selected_case_index',selected, ...
    'selected_requested_A_mm_s2',cases(selected,1),'selected_requested_J_mm_s3',cases(selected,2), ...
    'one_second_plateau_is_engineering_observation_choice',true, ...
    'existing_route_speed_guard_cm_s',20,'existing_route_value_guard_rpm',50, ...
    'existing_command_guard_rpm',commandGuard,'existing_feedback_guard_rpm',feedbackGuard, ...
    'required400_reference_rpm',speed/factor,'required400_command_budget_rpm',ceil(speed/factor)+followRpm, ...
    'existing_guards_accept400',false,'firmware_changed',false,'hardware_tested',false, ...
    'physical_speed_or_slip_or_safety_proven',false, ...
    'method','Local prepare/sample helpers reuse 20261111 MATLAB formulas, checked against active CHASSIS/USER/motion_profile.c seven phases; double precision kinematic model, not a bit-exact float C execution.', ...
    'requirement_classification',struct('R',{{'RUN-01','PWR-01','MECH-01'}}, ...
    'E',{{'MOT-02','MOT-04','LOC-01','LOC-02','LOC-05','LOC-07','RUN-06'}}, ...
    'Q','Existing official clarifications unchanged; 400 mm/s and 1 s plateau are not competition rules.'), ...
    'limitations','Reference reachability and RPM/deadline budget only; existing 20 cm/s, 50 reference RPM, 52 command and 80 feedback guards reject 400. No motor/drive/load/friction model, no physical motion, no new control fusion.');
writeJson(fullfile(outDir,'summary.json'),report);
fprintf('MATLAB_BASELINE40_OK cases=%d selected_A=%.0f selected_J=%.0f plateau_s=%.9f required_rpm=%.9f command_budget=%.0f\n', ...
    height(resultTable),cases(selected,1),cases(selected,2),rows(selected,7),speed/factor,ceil(speed/factor)+followRpm);
end

function number = macroNumber(source,name)
token = regexp(source,['(?m)^#define\s+' name '\s+([0-9.]+)[fU]?\s*$'],'tokens','once');
assert(~isempty(token),['Numeric macro not found: ' name]);
number = str2double(token{1});
end

function writeJson(path,value)
fid = fopen(path,'w','n','UTF-8');
assert(fid>=0,'Cannot write summary.');
cleanup = onCleanup(@() fclose(fid));
fwrite(fid,jsonencode(value),'char');
end

function [phase,duration,peak,endpointError] = prepareProfile(distance,limits)
speed=limits(1); accelLimit=limits(2); jerkLimit=limits(3);
tj=min(accelLimit/jerkLimit,sqrt(speed/jerkLimit));
ta=max(0,speed/(jerkLimit*tj)-tj);
minimumDistance=speed*(2*tj+ta); tv=0; peak=speed;
if distance<minimumDistance
    tj=accelLimit/jerkLimit;
    if distance>=2*accelLimit*tj^2
        ta=0.5*(sqrt(tj^2+4*distance/accelLimit)-3*tj);
    else
        tj=(distance/(2*jerkLimit))^(1/3); ta=0;
    end
    peak=jerkLimit*tj*(tj+ta);
else
    tv=(distance-minimumDistance)/speed;
end
assert(tj>0 && ta>=0 && tv>=0 && peak<=speed*1.00001);
phaseTime=[tj ta tj tv tj ta tj];
phaseJerk=[jerkLimit 0 -jerkLimit 0 -jerkLimit 0 jerkLimit];
phase=zeros(7,5); position=0; velocity=0; acceleration=0; time=0;
for k=1:7
    d=phaseTime(k); j=phaseJerk(k);
    phase(k,:)=[time position velocity acceleration j];
    position=position+velocity*d+0.5*acceleration*d^2+j*d^3/6;
    velocity=velocity+acceleration*d+0.5*j*d^2;
    acceleration=acceleration+j*d; time=time+d;
end
duration=time; endpointError=abs(position-distance);
assert(endpointError<=0.002+distance*1e-5);
end

function [position,velocity,acceleration,jerks] = sampleProfile(phase,time,distance,duration)
phaseTime=[phase(:,1);duration];
position=zeros(size(time)); velocity=zeros(size(time));
acceleration=zeros(size(time)); jerks=zeros(size(time));
for n=1:numel(time)
    if time(n)>=duration
        position(n)=distance; continue;
    end
    k=find(time(n)<phaseTime(2:end),1,'first');
    d=time(n)-phase(k,1);
    position(n)=phase(k,2)+phase(k,3)*d+0.5*phase(k,4)*d^2+phase(k,5)*d^3/6;
    velocity(n)=phase(k,3)+phase(k,4)*d+0.5*phase(k,5)*d^2;
    acceleration(n)=phase(k,4)+phase(k,5)*d;
    jerks(n)=phase(k,5);
end
end
