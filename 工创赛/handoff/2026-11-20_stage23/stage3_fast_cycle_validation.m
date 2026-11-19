function report = stage3_fast_cycle_validation()
% Offline schedule model. No target I/O, profile generation or firmware calls.
work=fileparts(mfilename('fullpath'));
source=fullfile(work,'stage3_adjust_recover_20261119','project');
user=fullfile(source,'CHASSIS','USER');
cycleSource=fileread(fullfile(user,'stage3_cycle.inc'));
ownerSource=fileread(fullfile(user,'integration10_complete.inc'));
params=fileread(fullfile(source,'USER','vehicle_parameters.h'));
assert(contains(cycleSource,'case 19U:S3_Axis(SPF_LIFT,s3_recovery_depth[target])'));
assert(contains(cycleSource,'now-s3_tray_tick)>=700U'));
assert(contains(ownerSource,'g_lift_ruler_controller.backoff_valid'));
assert(contains(ownerSource,'g_it_action==5U)?200U:600U'));
c.scale=macro(params,'SOURCE_LIFT_PULSES_PER_MM');
c.nominalBackoff=macro(params,'SOURCE_HOME_BACKOFF_PULSES');
c.nearTop=20*c.scale;
c.transitionZ=60795.02378472222-c.nearTop;
assert(abs(c.transitionZ-52795.02378472222)<1e-8);
c.grip=200; c.release=600; c.tray=700;
log=jsondecode(fileread(fullfile(work,'three-place-recover-watch-20261119-211951-936145.json')));
timing=analyze(log.samples);
normal=0;
positions=perms(1:3); orders=perms([1 2 5]);
for p=1:6
    for k=1:6
        for q=1:6
            for request32=[false true]
                for slow=[false true]
                    s=exercise(c,positions(p,:),orders(k,:),orders(q,:),request32,slow);
                    assert(s.placed==3 && s.recovered==3);
                    assert(isequal(s.returnedSlots,findSlots(orders(k,:),orders(q,:))));
                    normal=normal+1;
                end
            end
        end
    end
end
faultChecks=checkFaults(c);
report=struct('status','offline model passed; no firmware or physical acceptance','normal_cases',normal, ...
    'fault_checks',faultChecks,'candidate',c,'timing',timing, ...
    'validated_logical_cases',216,'timing_variants_per_case',4, ...
    'early_dm','after successful X/Z broadcast, a later same-transaction Z sample within 8000p permits DM prepare; X need not be zero', ...
    'transition','automatic mode1 only, final placement to first recovery: ring2 placement Z minus 8000p; not measured ground clearance', ...
    'stop_scope','modeled PWM, cache X/Z, broadcast, DM prepare and queue consumption boundaries; actual C transmission requires separate fixture evidence', ...
    'target_s',20,'target_met_verified',false);
fid=fopen(fullfile(work,'stage3_fast_cycle_validation.json'),'w'); assert(fid>=0);
fprintf(fid,'%s\n',jsonencode(report)); fclose(fid);
fprintf('Model passed %d synthetic schedules and %d guard/fault checks.\n',normal,faultChecks);
fprintf('Sample-derived baseline %.3f s; relaxed replay lower edge %.3f s; no 20 s acceptance.\n',timing.baseline_s,timing.relaxed_replay_lower_edge_s);
end

function value=macro(text,name)
t=regexp(text,['#define\s+' name '\s+([0-9.]+)'],'tokens','once');
assert(~isempty(t),name); value=str2double(t{1});
end

function slot=findSlots(colors,inventory)
[valid,slot]=ismember(colors,inventory);
assert(all(valid) && numel(unique(slot))==3);
end

function out=analyze(s)
if iscell(s), s=cellfun(@(x) struct('elapsed_s',x.elapsed_s, ...
        'g_stage3_align_done',x.g_stage3_align_done,'g_stage3_placed_count',x.g_stage3_placed_count, ...
        'g_stage3_state',x.g_stage3_state,'g_stage3_item',x.g_stage3_item,'g_stage3_phase',x.g_stage3_phase),s); end
t=[s.elapsed_s]; a=find([s.g_stage3_align_done]==1,1);
p=find([s.g_stage3_placed_count]==3,1); d=find([s.g_stage3_state]==6,1);
assert(~isempty(a) && a>1 && p>a && d>p);
out=struct('start_s',t(a),'three_placed_s',t(p),'done_s',t(d), ...
    'baseline_s',t(d)-t(a),'placement_s',t(p)-t(a),'recovery_s',t(d)-t(p), ...
    'median_sample_step_s',median(diff(t)), 'max_sample_step_s',max(diff(t)), ...
    'baseline_interval_s',[t(d-1)-t(a), t(d)-t(a-1)]);
% An intentionally optimistic replay: remove entire old X-tail, DM and tray
% segments whenever intended overlap could hide them. Do not model new travel
% speeds or treat shortened 70 mm motion as accepted. Boundary intervals can
% overlap, so this subtraction is an optimistic bound for this replay only.
span=zeros(0,5);
for item=0:2
    for pair=[13 29;22 23;23 24;24 26]'
        if pair(1)==13, l=a; r=p-1; else, l=p; r=d-1; end
        i=find([s(l:r).g_stage3_item]==item & [s(l:r).g_stage3_phase]==pair(1),1);
        j=find([s(l:r).g_stage3_item]==item & [s(l:r).g_stage3_phase]==pair(2),1);
        if isempty(i) || isempty(j) || i>=j, continue; end
        i=l+i-1; j=l+j-1;
        span(end+1,:)=[item pair' max(0,t(j-1)-t(i)) t(j)-t(i-1)]; %#ok<AGROW>
    end
end
out.replay_segments=span;
out.segment_columns={'placement_order_zero_based','start_phase','end_phase','min_observed_s','max_observed_s'};
out.relaxed_max_subtraction_s=sum(span(:,5));
out.relaxed_replay_lower_edge_s=max(0,out.baseline_interval_s(1)-out.relaxed_max_subtraction_s);
out.limit='Optimistic replay on the observed durations only, not a prediction or general motion bound; aliased controller events and new trajectory latency are unresolved.';
end

function s=exercise(c,rings,colors,inventory,request32,slow)
s=struct('phase',"place",'item',1,'placed',0,'recovered',0,'trayItem',0,'trayTick',-Inf, ...
    'returnedSlots',zeros(1,0),'xDone',false,'zDone',false,'yawSent',false,'yawDone',false, ...
    'emptyTransition',false,'activeSlot',0,'commands',zeros(1,4),'lastCommand',"",'tick',0,'releaseTick',0);
slots=findSlots(colors,inventory);
e=feedback(c);
s.auto=~request32;
for i=1:3
    e.now=e.now+1000; e.placeReleased=true; e.placeRing=rings(i);
    s=step(c,s,e,slots,rings); assert(s.lastCommand=="return_xz");
    assert(s.emptyTransition==(i==3 && ~request32));
    e.placeReleased=false; e.xDone=false; e.zDone=true;
    e.z=s.returnTarget; e.atTop=~s.emptyTransition; e.backoffRef=e.atTop;
    s=step(c,s,e,slots,rings); assert(s.phase=="place_return");
    e.xDone=true; e.zDone=false;
    s=step(c,s,e,slots,rings);
end
assert(s.placed==3 && s.phase=="placed");
assert(e.atTop==request32 && e.topRef && e.backoffRef==request32);
% Old placement prefetch with the same slot must still be discarded.
s.trayItem=slots(1); s.trayTick=e.now-10000;
e.beginRecovery=~request32; e.request32=request32;
s=step(c,s,e,slots,rings);
assert(s.phase=="ring" && s.trayItem==0 && isinf(s.trayTick));
e.beginRecovery=false; e.request32=false;
for i=1:3
    e.now=e.now+1000; e.ringReady=true; e.trayAccepted=true;
    s=step(c,s,e,slots,rings);
    assert(s.lastCommand=="lower_and_tray" && s.trayItem==slots(i));
    e.ringReady=false; e.trayAccepted=false; e.downDone=true;
    s=step(c,s,e,slots,rings); assert(s.lastCommand=="grip");
    e.downDone=false; e.now=e.now+c.grip-1; e.gripDone=true;
    s=step(c,s,e,slots,rings); assert(s.phase=="grip");
    e.now=e.now+1; e.z=c.nearTop;
    s=step(c,s,e,slots,rings); assert(s.lastCommand=="loaded_return_xz");
    e.gripDone=false; e.xDone=false; e.xZero=false; e.zDone=false;
    s=step(c,s,e,slots,rings); assert(~s.yawSent); % pre-broadcast sequence, even at 8000
    e.zSeq=e.zSeq+1;
    e.z=c.nearTop+1;
    s=step(c,s,e,slots,rings); assert(~s.yawSent);
    e.z=c.nearTop; e.freshZ=false;
    s=step(c,s,e,slots,rings); assert(~s.yawSent);
    e.freshZ=true; e.zTxn=e.zTxn+1;
    s=step(c,s,e,slots,rings); assert(~s.yawSent);
    e.zTxn=e.zTxn-1;
    s=step(c,s,e,slots,rings); assert(s.yawSent && ~s.xDone && s.lastCommand=="dm_prepare");
    count=s.commands;
    s=step(c,s,e,slots,rings); assert(isequal(count,s.commands) && ~s.yawDone);
    e.xDone=true; e.xZero=true;
    s=step(c,s,e,slots,rings); assert(s.yawSent && s.phase=="return");
    e.yawDone=true; e.zDone=true; e.z=e.safeZ; e.atTop=false;
    s=step(c,s,e,slots,rings); assert(s.phase=="return");
    if slow, e.now=s.trayTick+1400; else, e.now=s.trayTick+c.tray-1; end
    e.atTop=true;
    if i==1 && ~request32
        s=step(c,s,e,slots,rings); assert(s.phase=="return" && s.lowPermit);
    end
    e.backoffRef=true;
    s=step(c,s,e,slots,rings);
    if ~slow
        assert(s.phase=="return"); e.now=e.now+1;
        s=step(c,s,e,slots,rings);
    end
    assert(s.phase=="release" && s.lastCommand=="release_original_slot");
    e.releaseDone=true; e.now=s.releaseTick+c.release-1;
    s=step(c,s,e,slots,rings); assert(s.phase=="release");
    e.now=e.now+1;
    s=step(c,s,e,slots,rings);
    e.releaseDone=false; e.yawDone=false; e.zDone=false;
    assert(s.recovered==i && s.trayItem==0 && ~s.lowPermit);
end
end

function e=feedback(c)
e=struct('now',0,'stop',false,'topRef',true,'backoffRef',true,'feedbackValid',true,'xRef',true, ...
    'safeZ',c.nominalBackoff,'z',c.nominalBackoff,'atTop',true,'freshZ',true,'xDone',true,'xZero',true, ...
    'zDone',true,'yawDone',false,'ringReady',false,'trayAccepted',false,'downDone',false, ...
    'gripDone',false,'releaseDone',false,'placeReleased',false,'placeRing',0, ...
    'beginRecovery',false,'request32',false,'zSeq',1,'zTxn',1,'stopAt',"",'broadcastOK',true);
end

function s=step(c,s,e,slots,rings)
if any(s.phase==["done","fault"]), return; end
s.lastCommand="";
lowPermit=isfield(s,'lowPermit') && s.lowPermit;
if e.stop || ~e.topRef || (~e.backoffRef && ~lowPermit) || ~e.feedbackValid || ~e.xRef || ~isfinite(e.safeZ)
    s=halt(s); return;
end
switch s.phase
    case "place"
        if e.placeReleased && e.placeRing==rings(s.item)
            s.xDone=false; s.zDone=false; s.lastCommand="return_xz"; s.commands(1)=s.commands(1)+1;
            s.emptyTransition=s.item==3 && s.auto;
            s.lowPermit=s.emptyTransition;
            s.returnTarget=e.safeZ;
            if s.emptyTransition, s.returnTarget=c.transitionZ; end
            s=sendEvents(s,e,["cacheX","cacheZ","broadcast"]); if s.phase=="fault", return; end
            s.phase="place_return";
        end
    case "place_return"
        s.xDone=s.xDone || (e.xDone && e.xZero);
        if s.emptyTransition
            s.zDone=s.zDone || (e.zDone && e.freshZ && e.z==s.returnTarget && ~e.atTop && ~e.backoffRef);
        else
            s.zDone=s.zDone || (e.zDone && e.atTop);
        end
        if s.xDone && s.zDone
            s.placed=s.placed+1;
            if s.placed==3, s.phase="placed"; else, s.phase="place"; s.item=s.item+1; end
        end
    case "placed"
        if (s.auto && e.beginRecovery && s.emptyTransition && lowPermit) || (~s.auto && e.request32 && e.atTop && e.backoffRef)
            s.phase="ring"; s.item=1; s.trayItem=0; s.trayTick=-Inf;
        end
    case "ring"
        if e.ringReady && e.trayAccepted
            s=sendEvents(s,e,"pwm"); if s.phase=="fault", return; end
            s.trayItem=slots(s.item); s.trayTick=e.now; s.activeSlot=slots(s.item);
            s.lastCommand="lower_and_tray"; s.commands(2)=s.commands(2)+1; s.phase="lower";
        end
    case "lower"
        if e.downDone, s.tick=e.now; s.phase="grip"; s.lastCommand="grip"; end
    case "grip"
        if e.gripDone && e.now-s.tick>=c.grip
            s.xDone=false; s.zDone=false; s.yawSent=false; s.yawDone=false;
            s=sendEvents(s,e,["cacheX","cacheZ","broadcast"]); if s.phase=="fault", return; end
            s.broadcastZseq=e.zSeq; s.broadcastTxn=e.zTxn;
            s.phase="return"; s.lastCommand="loaded_return_xz";
        end
    case "return"
        s.xDone=s.xDone || (e.xDone && e.xZero); s.zDone=s.zDone || e.zDone;
        previouslySent=s.yawSent;
        if ~s.yawSent && e.freshZ && e.zTxn==s.broadcastTxn && e.zSeq>s.broadcastZseq && ...
                isfinite(e.z) && e.z>=0 && e.z<=c.nearTop
            s=sendEvents(s,e,["dmPrepare","dmQueueConsume"]); if s.phase=="fault", return; end
            s.yawSent=true; s.commands(3)=s.commands(3)+1; s.lastCommand="dm_prepare";
        end
        s.yawDone=s.yawDone || (previouslySent && e.yawDone);
        if s.xDone && e.xZero && s.zDone && e.atTop && e.backoffRef && s.yawDone && ...
                s.trayItem==s.activeSlot && s.trayTick>=0 && e.now-s.trayTick>=c.tray
            s.lowPermit=false;
            s.phase="release"; s.releaseTick=e.now; s.commands(4)=s.commands(4)+1;
            s.lastCommand="release_original_slot";
        end
    case "release"
        if e.releaseDone && e.now-s.releaseTick>=c.release
            s.recovered=s.recovered+1; s.returnedSlots(end+1)=s.activeSlot;
            s.trayItem=0; s.trayTick=-Inf;
            if s.recovered==3, s.phase="done"; else, s.phase="ring"; s.item=s.item+1; end
        end
end
end

function s=halt(s)
s.phase="fault"; s.trayItem=0; s.trayTick=-Inf; s.lastCommand=""; s.lowPermit=false;
end

function s=sendEvents(s,e,names)
if ~isfield(s,'sent'), s.sent=strings(1,0); end
for name=names
    if e.stopAt=="before_"+name || (name=="broadcast" && ~e.broadcastOK)
        s=halt(s); return;
    end
    s.sent(end+1)=name;
    if e.stopAt=="after_"+name, s=halt(s); return; end
end
end

function checks=checkFaults(c)
e=feedback(c); rr=[3 1 2]; slots=[2 3 1];
for phase=["place","place_return","placed","ring","lower","grip","return","release"]
    for failure=["stop","topRef","backoffRef","feedbackValid","xRef"]
        s=struct('phase',phase,'lastCommand',"queue",'trayItem',2,'trayTick',0,'commands',[1 2 3 4]);
        f=e; f.(failure)=failure=="stop";
        z=step(c,s,f,slots,rr);
        assert(z.phase=="fault" && z.trayItem==0 && z.lastCommand=="" && isequal(z.commands,s.commands));
    end
end
s=struct('phase',"return",'lastCommand',"",'trayItem',2,'trayTick',0,'commands',[1 2 3 4], ...
    'xDone',false,'zDone',false,'yawSent',false,'yawDone',false,'activeSlot',2,'broadcastZseq',0,'broadcastTxn',1);
f=e; f.xZero=false; f.atTop=true; f.yawDone=true; f.now=c.tray+100;
z=step(c,s,f,slots,rr); assert(z.phase=="return" && ~z.xDone);
assert(z.yawSent && ~z.yawDone);
checks=41;
groups={["pwm"],["cacheX","cacheZ","broadcast"],["dmPrepare","dmQueueConsume"]};
states={struct('phase',"ring",'item',1,'commands',[0 0 0 0]), ...
    struct('phase',"grip",'tick',0,'commands',[0 0 0 0]),s};
for group=1:numel(groups)
    names=groups{group}; base=states{group};
    f=e; f.ringReady=true; f.trayAccepted=true; f.now=c.grip; f.gripDone=true;
    for j=1:numel(names)
        for after=[false true]
            if after, prefix="after_"; else, prefix="before_"; end
            f.stopAt=prefix+names(j);
            halted=step(c,base,f,slots,rr);
            assert(halted.phase=="fault" && halted.trayItem==0 && ~halted.lowPermit);
            assert(numel(halted.sent)==j-1+after && all(halted.sent==names(1:j-1+after)));
            assert(isequaln(halted,step(c,halted,e,slots,rr)));
            checks=checks+1;
        end
    end
end
f.stopAt=""; f.broadcastOK=false;
halted=step(c,states{2},f,slots,rr);
assert(halted.phase=="fault" && isequal(halted.sent,["cacheX","cacheZ"]) && ~isfield(halted,'broadcastZseq'));
f.trayAccepted=false;
held=step(c,states{1},f,slots,rr);
assert(held.phase=="ring" && ~isfield(held,'trayTick'));
checks=checks+2;
end
