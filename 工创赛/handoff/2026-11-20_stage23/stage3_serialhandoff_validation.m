% Offline contract: every post uses one owner axis and matching DONE.
here=fileparts(mfilename('fullpath'));
base=jsondecode(fileread(fullfile(here,'stage3_standalone_validation.json')));
cfg=base.config;
assert(base.passed && cfg.mode==1 && isequal(cfg.mission_rings(:)',[2 1 3]));
orders=perms(1:3); goals=zeros(6,1); n=zeros(1,3);
for i=1:6
    order=orders(i,:);
    goals(i)=cfg.z_place(order(3))-16000;
    for commandTick=[0 500 2^32-80]
        pwm=commandTick+23;
        init=struct('phase',0,'pwm',NaN,'post',[],'target',[],'failed',false);
        a=step(init,'poll',tick(pwm+1000),cfg,order);
        assert(isempty(a.post));
        a=step(init,'pwm_ok',tick(pwm),cfg,order);
        b=step(a,'poll',tick(pwm+99),cfg,order);
        assert(b.phase==0 && isempty(b.post));
        z=step(b,'poll',tick(pwm+100),cfg,order);
        assert(z.phase==1 && isequal(z.post,4) && z.target==goals(i));
        % Masks: DM=1, X=2, Z=4. One mask per post; no group broadcast.
        w=step(z,'old_z_done',tick(pwm+101),cfg,order);
        w=step(w,'x_done',tick(pwm+101),cfg,order);
        w=step(w,'poll',tick(pwm+999),cfg,order);
        assert(isequal(w.post,4));
        x=step(w,'z_done',tick(pwm+1000),cfg,order);
        assert(isequal(x.post,[4 2]) && x.target(end)==0);
        w=step(x,'old_x_done',tick(pwm+1001),cfg,order);
        w=step(w,'z_done',tick(pwm+1001),cfg,order);
        w=step(w,'poll',tick(pwm+1999),cfg,order);
        assert(isequal(w.post,[4 2]));
        d=step(w,'x_done',tick(pwm+2000),cfg,order);
        assert(d.phase==3 && isequal(d.post,[4 2 1]) && d.target(end)==cfg.yaw(order(1)));
        n(1)=n(1)+1;
        for failure={'stop','fault','timeout'}
            for pending={b,z,x}
                q=step(pending{1},failure{1},tick(pwm+100),cfg,order);
                q=step(q,'poll',tick(pwm+100),cfg,order);
                q=step(q,'z_done',tick(pwm+101),cfg,order);
                q=step(q,'x_done',tick(pwm+102),cfg,order);
                assert(q.failed && isequal(q.post,pending{1}.post));
                n(2)=n(2)+1;
            end
        end
        q=step(init,'pwm_reject',tick(pwm),cfg,order);
        q=step(q,'poll',tick(pwm+100),cfg,order);
        assert(q.failed && isempty(q.post));
        n(3)=n(3)+1;
    end
end
report=struct('passed',true,'matlab',version,'model_only',true,'physical_acceptance',false, ...
    'dwell_ms',100,'rise_virtual_pulses',16000,'post_axis_masks',[4 2 1],'orders',orders, ...
    'target_virtual_pulses',goals,'success_cases',n(1),'stop_fault_timeout_cases',n(2), ...
    'pwm_reject_cases',n(3),'total_cases',sum(n),'inherited_standalone',base.cases, ...
    'inherited_final_zero',base.finalzero_cases_inherited);
f=fopen(fullfile(here,'stage3_serialhandoff_validation.json'),'w'); assert(f>=0);
fprintf(f,'%s\n',jsonencode(report)); fclose(f);
fprintf('PASS Stage3 serial handoff: %d cases; six ring orders; axis masks Z4 -> X2 -> DM1.\n',sum(n));

function s=step(s,event,now,cfg,order)
if s.failed, return; end
if any(strcmp(event,{'stop','fault','timeout','pwm_reject'})), s.failed=true; return; end
axis=0;
if s.phase==0
    if strcmp(event,'pwm_ok'), s.pwm=now; end
    if strcmp(event,'poll') && tick(now-s.pwm)>=100
        axis=4; target=cfg.z_place(order(3))-16000;
    end
elseif s.phase==1 && strcmp(event,'z_done')
    axis=2; target=0;
elseif s.phase==2 && strcmp(event,'x_done')
    axis=1; target=cfg.yaw(order(1));
end
if axis
    s.phase=s.phase+1; s.post(end+1)=axis; s.target(end+1)=target;
end
end
function t=tick(t)
t=mod(t,2^32);
end
