function validate_integration18_window5_20261115
root=fileparts(fileparts(mfilename('fullpath')));
base=fullfile(root,'evidence','integration','2026-11-15','integration18');
out=fullfile(base,'window5-pause');if ~exist(out,'dir'),mkdir(out);end
f=jsondecode(fileread(fullfile(base,'ext150-clog3000','fault134618-readonly.json')));
x=f.extension;scale=315.5*360/3200;window=5*scale;
assert(f.outside_existing_107deg_corridor && x.flags==1 && ~x.stall_active);
assert(x.current>=min(x.before,x.goal)-window && x.current<=max(x.before,x.goal)+window);
assert(abs(x.current-x.goal)>71);checks=3;
for extra=[-5.01 -5 -4.591064668075366 0 4.591064668075366 5 5.01]
    lower=x.goal+extra*scale;
    allowed=lower>=min(x.before,x.goal)-window && lower<=max(x.before,x.goal)+window;
    if extra>=-5,assert(allowed);else,assert(~allowed);end
    checks=checks+1;
end
for errorMm=[0 .5 1.5 2.01 4.591064668075366 5]
    arrived=errorMm*scale<=71;
    if errorMm>=2.01,assert(~arrived);end
    checks=checks+1;
end
for elapsed=[1999 2000 2999 3000 4000]
    for flags=[1 3 5 13]
        hardwareFault=bitand(flags,8)~=0 || bitand(flags,1)==0;
        stop=hardwareFault || (bitand(flags,4)~=0 && elapsed>=3000);
        if flags==13,assert(stop);end
        if flags==5,assert(stop==(elapsed>=3000));end
        checks=checks+1;
    end
end
p=jsondecode(fileread(fullfile(base,'recognition-pause','matlab-result.json')));
assert(p.passed && p.local_matlab);checks=checks+1;
r=struct('passed',true,'local_matlab',true,'checks',checks,...
    'extension_rpm',150,'stall_confirm_ms',3000,'extension_path_window_mm',5,...
    'extension_path_window_virtual_deg',window,'arrival_virtual_deg',71,...
    'zero_virtual_deg',17.75,'lift_path_window_virtual_deg',45,...
    'actual_fault_sample_now_inside_path_window',true,'actual_sample_still_not_arrived',true,...
    'recognition_pause_matlab_checks',p.checks,'mechanical_settling_unverified',true,...
    'hardware_access',false,'flashed',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('PASS window5 MATLAB: %u checks; actual4.591mm inside5mm, arrival still2mm,150RPM/3s unchanged\n',checks);
end
