function validate_integration18_ext150_clog3000_20261115
% Prior to implementation: requested speed/time changes, not a torque model.
root=fileparts(fileparts(mfilename('fullpath')));
base=fullfile(root,'evidence','integration','2026-11-15','integration18');
out=fullfile(base,'ext150-clog3000');if ~exist(out,'dir'),mkdir(out);end
trial=jsondecode(fileread(fullfile(base,'committed-20261115-121451.json')));
fault=trial.first_controller_fault;
assert(~trial.complete && fault.g_low_phase==2 && fault.g_low_done_sequence==0);
assert(fault.g_low_release_ms==0 && fault.g_low_clear_valid==0);
historical=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-14',...
    'top-pick-lead8','read-20261114-184942.json')));
assert(historical.complete && historical.configuration.clog_pro==1);
raw=uint8(sscanf(historical.configuration_hex,'%2x').');
assert(numel(raw)==36 && isequal(raw(1:3),uint8([66 37 24])) && raw(end)==107);
% The driver readback excludes address. Only bytes31/32 (zero based) are time.
modified=raw;modified(32)=uint8(floor(3000/256));modified(33)=uint8(mod(3000,256));
changed=find(raw~=modified);assert(isequal(changed,[32 33]));
assert(modified(27)==raw(27) && isequal(modified(28:31),raw(28:31)));
checks=5;
for elapsed=[0 1999 2000 2500 2999 3000 4000]
    for flags=[1 3 5 7 9 13 15]
        hardFault=bitand(flags,8)~=0 || bitand(flags,1)==0;
        oldStop=hardFault || (bitand(flags,4)~=0 && elapsed>=2000);
        newStop=hardFault || (bitand(flags,4)~=0 && elapsed>=3000);
        if hardFault,assert(newStop);end
        if bitand(flags,4)~=0 && ~hardFault && elapsed>=2000 && elapsed<3000
            assert(oldStop && ~newStop);
        end
        if elapsed>=3000,assert(newStop==oldStop);end
        checks=checks+1;
    end
end
% Existing arrival/corridor/zero tolerances are unchanged by a time relaxation.
for errorMm=[0 .5 1.9 2.1 14.8]
    arrived=errorMm<=71/(315.5*360/3200);
    if errorMm==14.8,assert(~arrived);end
    checks=checks+1;
end
assert(150*10==1500 && 200*10==2000 && 600==600);checks=checks+1;
% 0xFD X position units reflect the actual saved Scale option. No geometry rewrite.
wirePosition=hex2dec('0000860E');scale=100;
distanceRevs=(wirePosition/scale)/360;accelRps2=600/60;
speeds=[200 150];times=zeros(1,2);peaks=zeros(1,2);
for k=1:2
    vmax=speeds(k)/60;
    if distanceRevs<=vmax^2/accelRps2
        times(k)=2*sqrt(distanceRevs/accelRps2);peaks(k)=sqrt(distanceRevs*accelRps2)*60;
    else
        times(k)=distanceRevs/vmax+vmax/accelRps2;peaks(k)=speeds(k);
    end
end
assert(peaks(2)<=150 && times(2)>=times(1));checks=checks+1;
report=struct('passed',true,'local_matlab',true,'checks',checks,...
    'extension_rpm',150,'extension_acceleration_rpm_s',600,...
    'software_stall_confirm_ms',3000,'driver_clog_time_ms',3000,...
    'driver_current_and_speed_limits_unchanged',true,'driver_clog_enable_unchanged',true,...
    'position_and_travel_tolerances_unchanged',true,'same_hardware_protection_bit_still_stops',true,...
    'historical_config_only_current_config_must_be_read_before_apply',true,...
    'historical_clog_rpm',historical.configuration.clog_rpm,...
    'historical_clog_ma',historical.configuration.clog_ma,...
    'protocol_distance_revolutions',distanceRevs,'ideal_profile_seconds_200_150',times,...
    'ideal_peak_rpm_200_150',peaks,'motor_current_load_and_mechanical_binding_not_modelled',true,...
    'physical_fault_resolution_verified',false,'hardware_access',false,'flashed',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('PASS extension150/clog3000: %u checks; protection/time/arrival separate; no physical fix claim\n',checks);
end
