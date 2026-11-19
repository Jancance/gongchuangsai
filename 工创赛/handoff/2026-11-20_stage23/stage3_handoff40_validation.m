% Specification model; feedback events stand for matching existing owner DONEs.
here=fileparts(mfilename('fullpath'));
profile=jsondecode(fileread(fullfile(here,'stage3_anticipatory_finalzero_20261120','recovery-profile-candidate.json')));
r=profile.rings;
depth=[r.x1.release_depth_pulses r.x2.release_depth_pulses r.x3.release_depth_pulses];
yaw=[r.x1.yaw_deg r.x2.yaw_deg r.x3.yaw_deg];
orders=perms(1:3); z=zeros(6,1); n=zeros(1,4);
for i=1:size(orders,1)
    order=orders(i,:); last=order(3); first=order(1);
    z(i)=depth(last)-16000;
    assert(z(i)>0 && depth(last)-z(i)==16000 && z(i)~=depth(2)-8000);
    if last~=2, assert(z(i)~=depth(2)-16000); end
    s=struct('stage',0,'pwm',NaN,'done',false(1,2),'post',[], ...
        'finish',NaN,'item',1,'order',order,'z',NaN,'target',NaN);
    s=step(s,'pwm_ok',31,depth,yaw);
    s=step(s,'poll',130,depth,yaw);
    assert(s.stage==0 && isempty(s.post) && isnan(s.target));
    s=step(s,'poll',131,depth,yaw);
    assert(s.stage==1 && isequal(s.post,[1 2]) && s.z==z(i) && s.target==0);
    for arrival={'x','z';'z','x'}
        a=step(s,'old_done',135,depth,yaw);
        a=step(a,arrival{1},140,depth,yaw);
        a=step(a,arrival{1},141,depth,yaw);
        a=step(a,'poll',900,depth,yaw);
        assert(a.stage==1 && isequal(a.post,[1 2]));
        b=step(a,arrival{2},901,depth,yaw);
        assert(b.stage==2 && isequal(b.post,[1 2 3]) && b.target==yaw(first));
        n(1)=n(1)+1;
        for event={'stop','fault','timeout'}
            for q={s,a}
                c=step(q{1},event{1},901,depth,yaw);
                c=step(c,arrival{2},902,depth,yaw);
                assert(c.stage==-1 && isequal(c.post,[1 2]) && isnan(c.finish));
                n(2)=n(2)+1;
            end
        end
    end
    for item=1:3
        now=1000+item*500;
        b=step(b,'slot_ready',now,depth,yaw);
        assert(b.stage==3 && b.post(end)==4 && b.item==item); % immediate PWM request
        p=b.post;
        c=step(b,'poll',now+400,depth,yaw); % PWM has not succeeded
        assert(c.stage==3 && isequal(c.post,p) && isnan(c.finish));
        c=step(b,'pwm_reject',now+1,depth,yaw);
        c=step(c,'poll',now+400,depth,yaw);
        assert(c.stage==-1 && isequal(c.post,p) && isnan(c.finish));
        b=step(b,'pwm_ok',now+13,depth,yaw);
        b=step(b,'poll',now+112,depth,yaw);
        assert(b.stage==3 && isequal(b.post,p) && isnan(b.finish));
        b=step(b,'poll',now+113,depth,yaw);
        assert(isequal(b.post,[p 3]) && isnan(b.finish));
        if item<3
            assert(b.stage==2 && b.item==item+1 && b.target==yaw(order(item+1)));
        else
            assert(b.stage==4 && b.target==0);
            b=step(b,'old_done',now+114,depth,yaw);
            assert(b.stage==4 && isnan(b.finish));
            b=step(b,'dm_done',now+115,depth,yaw);
            assert(b.stage==6 && b.finish==now+115);
        end
        n(3)=n(3)+1; n(4)=n(4)+2;
    end
end
report=struct('passed',true,'matlab',version,'model_only',true,'physical_acceptance',false, ...
    'dwell_ms',100,'rise_mm',40,'rise_virtual_pulses',16000,'orders',orders, ...
    'target_virtual_pulses',z,'ring_release_depths',depth,'arrival_order_cases',n(1), ...
    'stop_fault_timeout_cases',n(2),'return_dwell_cases',n(3),'no_successful_pwm_cases',n(4),'total_cases',sum(n));
f=fopen(fullfile(here,'stage3_handoff40_validation.json'),'w'); assert(f>=0);
fprintf(f,'%s\n',jsonencode(report)); fclose(f);
fprintf('PASS Stage3 handoff 40mm: %d cases across all six ring orders.\n',sum(n));

function s=step(s,event,now,depth,yaw)
if s.stage<0 || s.stage==6, return; end
if any(strcmp(event,{'stop','fault','timeout','pwm_reject'})), s.stage=-1; return; end
switch s.stage
    case 0
        if strcmp(event,'pwm_ok'), s.pwm=now; end
        if strcmp(event,'poll') && now-s.pwm>=100
            s.stage=1; s.post=[1 2]; s.target=0; s.z=depth(s.order(3))-16000;
        end
    case 1
        if strcmp(event,'x'), s.done(1)=true; end
        if strcmp(event,'z'), s.done(2)=true; end
        if all(s.done), s.stage=2; s.post(end+1)=3; s.target=yaw(s.order(1)); end
    case 2
        if strcmp(event,'slot_ready'), s.stage=3; s.pwm=NaN; s.post(end+1)=4; end
    case 3
        if strcmp(event,'pwm_ok'), s.pwm=now; end
        if strcmp(event,'poll') && now-s.pwm>=100
            s.post(end+1)=3;
            if s.item<3
                s.item=s.item+1; s.target=yaw(s.order(s.item)); s.stage=2;
            else
                s.target=0; s.stage=4;
            end
        end
    case 4
        if strcmp(event,'dm_done'), s.stage=6; s.finish=now; end
end
end
