% Specification model for compiled defaults and independent two-board boot.
here=fileparts(mfilename('fullpath'));
src=fullfile(here,'stage3_anticipatory_handoff40_20261120');
p=jsondecode(fileread(fullfile(src,'recovery-profile-candidate.json')));
v=jsondecode(fileread(fullfile(src,'stage3-vision-13-5.json')));
c=v.ring_row_alignment.integrated_compensation;
r=[p.rings.x1 p.rings.x2 p.rings.x3];
cfg=struct('mode',1,'count',3,'task_batch',1,'slots',[2 1 5], ...
    'mission_rings',[2 1 3],'align_mm',2,'align_deg',1,'pi_forward_right_mm',[c.forward_mm c.right_mm], ...
    'carrier_deg',p.carrier_pick.yaw_deg,'pick_pulses',p.carrier_pick.depth_pulses, ...
    'yaw',[r.yaw_deg],'x',[r.extension_mm],'z_place',[r.release_depth_pulses], ...
    'z_recover',[r.recovery_depth_pulses]);
assert(isequal(cfg.pi_forward_right_mm,[13 5]) && isequal(cfg.x,[43 0 70.5]));
assert(cfg.carrier_deg==36.5 && cfg.pick_pulses==8003.47);
assert(max(abs(cfg.z_place-[61802.22274305556 60795.02378472222 63002.22274305556]))<1e-8);
assert(max(abs(cfg.z_recover-[61802.22274305556 61595.02378472222 63002.22274305556]))<1e-8);
h=jsondecode(fileread(fullfile(here,'stage3_handoff40_validation.json')));
f=jsondecode(fileread(fullfile(here,'stage3_finalzero_validation.json')));
assert(h.passed && f.passed && h.dwell_ms==100 && f.dwell_ms==100 && f.target_deg==0);
i=find(all(h.orders==cfg.mission_rings,2),1);
assert(~isempty(i) && abs(h.target_virtual_pulses(i)-(cfg.z_place(3)-16000))<1e-8);
cases=0;
for piFirst=[false true]
    s=boot(cfg);
    if piFirst, s=piBoot(s); end
    assert(s.state==0 && s.req==0 && s.starts==0 && ~s.motion && isequal(s.config,cfg));
    if ~piFirst, s=piBoot(s); end
    assert(s.state==0 && s.req==0 && s.starts==0 && ~s.motion && s.observe);
    cases=cases+1;
    s=key(s,true); assert(s.req==1 && s.state==0 && ~s.motion);
    s=service(s); assert(s.state==1 && s.starts==1 && isequal(s.runConfig,cfg));
    for t=1:20, s=service(key(s,true)); end
    assert(s.starts==1 && s.stops==0 && s.req==0);
    s.state=5;
    for n=1:3
        s=cycleEvent(s,'placed',cfg.mission_rings(n));
        assert(s.state==5 && s.placed==n && s.recovered==0);
    end
    for n=1:3
        s=cycleEvent(s,'recovered',cfg.mission_rings(n));
        assert(s.state==5 && s.recovered==n && s.zero==(n==3));
    end
    assert(~s.countEnded);
    s=cycleEvent(s,'dm_zero_done',0);
    assert(s.state==6 && s.countEnded && ~s.motion);
    for t=1:20, s=service(key(s,true)); end
    assert(s.starts==1 && s.req==0 && s.state==6);
    s=key(key(s,false),true); assert(s.req==1);
    s=service(s); assert(s.starts==2 && s.state==1 && isequal(s.config,cfg));
    cases=cases+1;
    s=boot(s.config); assert(s.starts==0 && s.state==0 && isequal(s.config,cfg));
    cases=cases+1;
end
for state=[1 2 3 4 5 7 8 9 10 11]
    s=boot(cfg); s.state=state;
    s=key(s,true);
    if state==9
        assert(s.req==0 && s.state==9);
    else
        assert(s.req==9);
        s=service(s); assert(s.state==9 && s.stops==1);
    end
    s=service(key(key(s,false),true));
    assert(s.state==9 && s.req==0 && s.starts==0 && ~s.motion);
    cases=cases+1;
end
report=struct('passed',true,'matlab',version,'model_only',true,'physical_acceptance',false, ...
    'cases',cases,'config',cfg,'handoff_cases_inherited',h.total_cases, ...
    'finalzero_cases_inherited',f.total_cases);
out=fopen(fullfile(here,'stage3_standalone_validation.json'),'w'); assert(out>=0);
fprintf(out,'%s\n',jsonencode(report)); fclose(out);
fprintf('PASS Stage3 standalone: %d scenarios; inherited handoff40=%d and finalzero=%d.\n',cases,h.total_cases,f.total_cases);

function s=boot(cfg)
s=struct('config',cfg,'runConfig',[],'state',0,'req',0,'pressed',false, ...
    'starts',0,'stops',0,'motion',false,'observe',false,'placed',0,'recovered',0,'zero',false,'countEnded',false);
end
function s=piBoot(s)
s.observe=true;
end
function s=key(s,pressed)
% Sampled one-shot press event; no assumed debounce period or repeat delay.
edge=pressed && ~s.pressed; s.pressed=pressed;
if ~edge || s.state==9, return; end
if any(s.state==[0 6]), s.req=1; else, s.req=9; end
end
function s=service(s)
if s.req==1
    s.state=1; s.starts=s.starts+1; s.runConfig=s.config;
    s.placed=0; s.recovered=0; s.zero=false; s.countEnded=false;
elseif s.req==9
    s.state=9; s.stops=s.stops+1; s.motion=false;
end
s.req=0;
end
function s=cycleEvent(s,event,value)
assert(s.state==5);
if strcmp(event,'placed')
    assert(s.recovered==0 && value==s.runConfig.mission_rings(s.placed+1));
    s.placed=s.placed+1;
elseif strcmp(event,'recovered')
    assert(s.placed==s.runConfig.count && value==s.runConfig.mission_rings(s.recovered+1));
    s.recovered=s.recovered+1; s.zero=s.recovered==s.runConfig.count;
elseif strcmp(event,'dm_zero_done')
    assert(s.zero && value==0);
    s.state=6; s.countEnded=true; s.motion=false;
end
end
