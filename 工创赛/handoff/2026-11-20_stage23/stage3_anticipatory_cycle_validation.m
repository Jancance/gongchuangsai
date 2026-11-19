function report=stage3_anticipatory_cycle_validation()
% Offline discrete-event specification. No driver or target I/O.
work=fileparts(mfilename('fullpath'));
parent=fullfile(work,'stage3_yaw1_handoff3_20261120');
m=jsondecode(fileread(fullfile(parent,'stage3-yaw1-handoff3-compatible.json')));
p=jsondecode(fileread(fullfile(parent,'recovery-profile-candidate.json')));
assert(m.stage3_strict_align.accept_norm_um==1500 && m.stage3_strict_align.accept_abs_yaw_mdeg==1000);
cfg=jsondecode(fileread(fullfile(parent,'stage3-vision-13-5.json')));
v=cfg.ring_row_alignment.integrated_compensation;
assert(v.forward_mm==13 && v.right_mm==5 && ~cfg.calibration_verified);
c.pick=p.carrier_pick.depth_pulses; c.safeZ=600; c.near=2000;
c.handoff=m.stage3_empty_handoff.depth_pulses;
for ring=1:3
    r=p.rings.(sprintf('x%d',ring)); c.x(ring)=r.extension_mm;
    c.z(ring)=r.release_depth_pulses; c.yaw(ring)=r.yaw_deg; c.rz(ring)=r.recovery_depth_pulses;
end
ringOrder=perms(1:3); colors=perms([1 2 5]); cases=0; blocked=0;
for i=1:6
    for j=1:6
        for k=1:6
            [a,b]=run(c,ringOrder(i,:),colors(j,:),colors(k,:));
            cases=cases+1; blocked=blocked+b;
            assert(isequal(a.xEnd,c.x(ringOrder(i,:))) && isequal(a.placeZ,c.z(ringOrder(i,:))));
            assert(isequal(a.recoverZ,c.rz(ringOrder(i,:))));
            assert(isequal(a.handoff,[c.handoff c.yaw(ringOrder(i,1)) 0]));
            assert(isequal(colors(k,a.returnSlots),colors(j,:)));
        end
    end
end
stopCases=0;
for phase=["pick_slot","pick_open","pick","carry_up","carry_down","release","empty_handoff","recover_down","recover_grip","recover_up","tray_release"]
    for after=[false true]
        s=init(phase); e=event(); e.stop=true; s.emitted=double(after);
        z=step(s,e,c);
        assert(z.phase=="fault" && z.stopMask==7 && z.emitted==s.emitted && z.closeTick==-Inf);
        assert(isequaln(z,step(z,event(),c)));
        stopCases=stopCases+1;
    end
end
s=init("carry_up"); e=event(); e.refs=false;
s=step(s,e,c); assert(s.phase=="fault" && s.stopMask==7);
report=struct('passed',true,'mapped_schedules',cases,'blocked_gate_probes',blocked, ...
    'stop_cases',stopCases,'reference_loss_cases',1,'parent_image',m.images.Vehicle_LiftRuler.sha256, ...
    'mode',1,'slot_close_ms',100,'ring_open_ms',100,'recovery_close_ms',100, ...
    'tray_open_ms',100,'tray_settle_ms',0,'pick_tray_index_ms',0,'pick_preopen_ms',0, ...
    'near_mechanical_top_pulses',2000, ...
    'near_pick_remaining_pulses',2000,'near_ring_yaw_deg',3,'physical_acceptance',false, ...
    'payload_sensor_proof',false,'target_20s_verified',false, ...
    'scope','Event model only; no firmware or physical actuation, fixture-level send-boundary tests still required.');
fid=fopen(fullfile(work,'stage3_anticipatory_cycle_validation_final.json'),'w'); assert(fid>=0);
fprintf(fid,'%s\n',jsonencode(report)); fclose(fid);
fprintf('PASS anticipatory cycle: %d mappings, %d blocked gate probes, %d simulated STOP checks.\n',cases,blocked,stopCases);
end

function [trace,blocked]=run(c,rings,colors,slots)
trace=struct('xEnd',zeros(1,3),'placeZ',zeros(1,3),'recoverZ',zeros(1,3), ...
    'returnSlots',zeros(1,3),'handoff',[]);
blocked=0;
for k=1:3
    r=rings(k); s=init("pick_slot"); s.to=[c.x(r) c.z(r) c.yaw(r)];
    e=event(); e.pwmOK=false;
    assert(isequaln(s,step(s,e,c)) && ~isfinite(s.trayTick)); blocked=blocked+1;
    e.pwmOK=true; s=step(s,e,c); assert(s.phase=="pick_open" && s.trayTick==0);
    e.pwmOK=false; assert(isequaln(s,step(s,e,c))); blocked=blocked+1;
    e.pwmOK=true; s=step(s,e,c); assert(s.phase=="pick" && s.closeTick==-Inf && s.emitted==2);
    e=event(); e.z=c.pick-2001; e.xAtPick=true;
    assert(isequaln(s,step(s,e,c))); blocked=blocked+1;
    e.z=c.pick-2000; e.xAtPick=false;
    assert(isequaln(s,step(s,e,c))); blocked=blocked+1;
    e.xAtPick=true; e.freshZ=false;
    assert(isequaln(s,step(s,e,c))); blocked=blocked+1;
    e.freshZ=true; e.pwmOK=false;
    assert(isequaln(s,step(s,e,c))); blocked=blocked+1;
    e.pwmOK=true; e.now=23;
    s=step(s,e,c); assert(s.closeTick==23 && s.emitted==3);
    e.now=122; e.pickDone=true;
    assert(isequaln(s,step(s,e,c))); blocked=blocked+1;
    e.now=123; e.xAtPick=false;
    assert(isequaln(s,step(s,e,c))); blocked=blocked+1;
    e.xAtPick=true; e.pickDone=false;
    assert(isequaln(s,step(s,e,c))); blocked=blocked+1;
    e.pickDone=true; s=step(s,e,c); assert(s.phase=="carry_up" && s.zGoal==c.safeZ);
    e.z=2001; a=step(s,e,c); assert(~a.dmSent); blocked=blocked+1;
    e.z=2000; e.freshZ=false; a=step(s,e,c); assert(~a.dmSent); blocked=blocked+1;
    e.freshZ=true; e.dmSentOK=true; e.dmSeq=50; e.dmErr=0;
    s=step(s,e,c); assert(s.dmSent && s.phase=="carry_up");
    e.topDone=true; e.zReleased=true; e.dmDone=true;
    a=step(s,e,c); assert(a.phase=="carry_up"); blocked=blocked+1; % pre-send DM sample
    e.dmSeq=51; e.dmErr=3.001;
    a=step(s,e,c); assert(a.phase=="carry_up"); blocked=blocked+1;
    e.dmErr=3; e.zReleased=false;
    a=step(s,e,c); assert(a.phase=="carry_up" && a.zGoal==c.safeZ); blocked=blocked+1;
    e.zReleased=true; e.xAtPick=false;
    a=step(s,e,c); assert(a.phase=="carry_up"); blocked=blocked+1;
    e.xAtPick=true; e.dmDone=false;
    s=step(s,e,c); assert(s.phase=="carry_down" && s.zGoal==c.z(r));
    e.xDone=true; e.zDone=true;
    a=step(s,e,c); assert(a.phase=="carry_down"); blocked=blocked+1;
    e.dmDone=true; e.now=200;
    e.pwmOK=false; assert(isequaln(s,step(s,e,c))); blocked=blocked+1;
    e.pwmOK=true;
    s=step(s,e,c); assert(s.phase=="release" && s.releaseTick==200);
    e.now=299; a=step(s,e,c); assert(a.phase=="release"); blocked=blocked+1;
    e.now=300; s=step(s,e,c); assert(s.phase=="placed");
    trace.xEnd(k)=s.to(1); trace.placeZ(k)=s.zGoal;
end
% The low empty handoff uses the parent's independent Z, DM, X order.
s=init("empty_handoff"); e=event(); e.xDone=true; e.zDone=true;
a=step(s,e,c); assert(a.phase=="empty_handoff"); blocked=blocked+1;
e.dmDone=true; a=step(s,e,c); assert(a.phase=="recover_down");
trace.handoff=[c.handoff c.yaw(rings(1)) 0];
for k=1:3
    s=init("recover_down"); s.to(2)=c.rz(rings(k)); e=event(); e.now=200; e.pwmOK=false; e.zDone=true;
    assert(isequaln(s,step(s,e,c)) && ~isfinite(s.trayTick)); blocked=blocked+1;
    e.pwmOK=true; e.zDone=false;
    s=step(s,e,c); assert(s.trayTick==200);
    e.zDone=true; e.pwmOK=false; assert(isequaln(s,step(s,e,c))); blocked=blocked+1;
    e.pwmOK=true;
    s=step(s,e,c); assert(s.closeTick==200);
    e.now=299; a=step(s,e,c); assert(a.phase=="recover_grip"); blocked=blocked+1;
    e.now=300; s=step(s,e,c); assert(s.phase=="recover_up");
    e.z=2001; e.dmSentOK=true; a=step(s,e,c); assert(~a.dmSent); blocked=blocked+1;
    e.z=2000; s=step(s,e,c); assert(s.dmSent);
    e.dmSeq=1; e.dmDone=true; e.topDone=true; e.xDone=false;
    a=step(s,e,c); assert(a.phase=="recover_up"); blocked=blocked+1;
    e.xDone=true; e.pwmOK=false; a=step(s,e,c); assert(a.phase=="recover_up" && a.releaseTick==-Inf); blocked=blocked+1;
    e.pwmOK=true; s=step(s,e,c); assert(s.phase=="tray_release" && s.releaseTick==300 && s.trayTick==200);
    e.now=399; a=step(s,e,c); assert(a.phase=="tray_release"); blocked=blocked+1;
    e.now=400; s=step(s,e,c); assert(s.phase=="recovered");
    trace.recoverZ(k)=s.to(2); trace.returnSlots(k)=find(slots==colors(k),1);
end
end

function s=init(phase)
s=struct('phase',phase,'to',[0 0 0],'zGoal',NaN,'closeTick',-Inf,'releaseTick',-Inf, ...
    'trayTick',-Inf,'dmSeq',-Inf,'dmSent',false,'emitted',0,'stopMask',0);
end

function e=event()
e=struct('now',0,'z',NaN,'xAtPick',false,'freshZ',true,'pwmOK',true,'pickDone',false, ...
    'topDone',false,'zReleased',false,'dmSeq',0,'dmErr',Inf,'freshDM',true,'dmSentOK',false, ...
    'dmDone',false,'xDone',false,'zDone',false,'stop',false,'refs',true);
end

function s=step(s,e,c)
if s.phase=="fault", return; end
if e.stop || ~e.refs
    s.phase="fault"; s.stopMask=7; s.closeTick=-Inf; s.releaseTick=-Inf; s.trayTick=-Inf;
    s.dmSeq=-Inf; s.dmSent=false; return;
end
switch s.phase
    case "pick_slot"
        if e.pwmOK, s.trayTick=e.now; s.phase="pick_open"; s.emitted=s.emitted+1; end
    case "pick_open"
        if isfinite(s.trayTick) && e.pwmOK, s.phase="pick"; s.emitted=s.emitted+1; end
    case "pick"
        remaining=c.pick-e.z;
        if s.closeTick==-Inf && e.freshZ && remaining>=0 && remaining<=c.near && e.xAtPick && e.pwmOK
            s.closeTick=e.now; s.emitted=s.emitted+1;
        end
        if isfinite(s.closeTick) && e.now-s.closeTick>=100 && e.pickDone && e.xAtPick
            s.phase="carry_up"; s.zGoal=c.safeZ; s.emitted=s.emitted+1;
        end
    case "carry_up"
        if ~s.dmSent && e.freshZ && e.z>=0 && e.z<=c.near && e.dmSentOK
            s.dmSent=true; s.dmSeq=e.dmSeq; s.emitted=s.emitted+1;
        end
        if s.dmSent && e.freshDM && e.dmSeq>s.dmSeq && abs(e.dmErr)<=3 && ...
                e.topDone && e.zReleased && e.xAtPick
            s.phase="carry_down"; s.zGoal=s.to(2); s.emitted=s.emitted+1;
        end
    case "carry_down"
        if e.xDone && e.zDone && e.dmDone && e.pwmOK
            s.phase="release"; s.releaseTick=e.now; s.emitted=s.emitted+1;
        end
    case "release"
        if e.now-s.releaseTick>=100, s.phase="placed"; end
    case "empty_handoff"
        if e.xDone && e.zDone && e.dmDone, s.phase="recover_down"; end
    case "recover_down"
        if s.trayTick==-Inf && e.pwmOK, s.trayTick=e.now; s.emitted=s.emitted+1; end
        if isfinite(s.trayTick) && e.zDone && e.pwmOK
            s.phase="recover_grip"; s.closeTick=e.now; s.emitted=s.emitted+1;
        end
    case "recover_grip"
        if e.now-s.closeTick>=100, s.phase="recover_up"; end
    case "recover_up"
        if ~s.dmSent && e.freshZ && e.z>=0 && e.z<=c.near && e.dmSentOK
            s.dmSent=true; s.dmSeq=e.dmSeq; s.emitted=s.emitted+1;
        end
        if s.dmSent && e.freshDM && e.dmSeq>s.dmSeq && e.dmDone && e.topDone && e.xDone && ...
                isfinite(s.trayTick) && e.pwmOK
            s.phase="tray_release"; s.releaseTick=e.now; s.emitted=s.emitted+1;
        end
    case "tray_release"
        if e.now-s.releaseTick>=100, s.phase="recovered"; end
end
end
