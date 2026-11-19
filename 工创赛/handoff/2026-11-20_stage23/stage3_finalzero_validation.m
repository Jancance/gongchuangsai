% Offline specification model. DM events represent the existing IT/DM owner.
here=fileparts(mfilename('fullpath'));
n=zeros(1,5);
for start=[0 2300 2^32-50]
    for delta=[0 1 41]
        applied=start+delta;
        for pollDelay=[0 1 7]
            due=applied+100+pollDelay;
            s=step(initial(1,3),'pwm_pending',tick(start));
            s=step(s,'pwm_ok',tick(applied));
            s=step(s,'poll',tick(applied+99));
            assert(s.phase==26 && s.posts==0 && s.state==5 && isnan(s.finish));
            n(2)=n(2)+1;
            r=step(s,'poll',tick(due));
            assert(r.phase==31 && r.posts==1 && r.target==0 && r.state==5);
            assert(isnan(r.finish) && r.recovered==3);
            for dmDelay=[1 13 300]
                q=step(r,'dm_busy',tick(due+dmDelay-1));
                q=step(q,'stale_done',tick(due+dmDelay-1));
                assert(q.posts==1 && q.phase==31 && q.state==5 && isnan(q.finish));
                q=step(q,'dm_done',tick(due+dmDelay));
                assert(q.phase==32 && q.state==6 && q.recovered==3);
                assert(q.finish==tick(due+dmDelay) && q.posts==1);
                q=step(q,'dm_done',tick(due+dmDelay+1));
                assert(q.finish==tick(due+dmDelay) && q.posts==1);
                n(1)=n(1)+1;
            end
        end
    end
end
for item=1:3
    for mode=[0 1 2]
        if mode==1 && item==3, continue; end
        s=step(initial(mode,item),'pwm_ok',100);
        s=step(s,'poll',200);
        assert(s.posts==0 && s.recovered==item && isnan(s.finish));
        if item<3, assert(s.phase==17 && s.state==5); else, assert(s.state==6); end
        n(3)=n(3)+1;
    end
end
for event={'pwm_reject','pwm_pending','stale_done'}
    s=step(initial(1,3),event{1},101);
    s=step(s,'poll',1000);
    assert(s.posts==0 && s.recovered==2 && s.state~=6 && isnan(s.finish));
    n(4)=n(4)+1;
end
for event={'stop','owner_fault','owner_timeout'}
    for boundary=1:4
        s=initial(1,3);
        if boundary>=2, s=step(s,'pwm_ok',101); end
        if boundary>=3, s=step(s,'poll',201); end
        if boundary>=4, s=step(s,'dm_busy',201); end
        % Failures have priority over same-tick dwell expiry or completion.
        s=step(s,event{1},201);
        s=step(s,'poll',201);
        s=step(s,'dm_done',201);
        assert(s.stopPath && s.state~=6 && isnan(s.finish));
        assert(s.posts==double(boundary>=3));
        n(5)=n(5)+1;
    end
end
report=struct('passed',true,'matlab',version,'model_only',true,'physical_acceptance',false, ...
    'dwell_ms',100,'target_deg',0,'success_cases',n(1),'dwell_boundary_cases',n(2), ...
    'unaffected_cases',n(3),'no_successful_pwm_cases',n(4), ...
    'fault_timeout_stop_cases',n(5),'total_cases',sum(n));
f=fopen(fullfile(here,'stage3_finalzero_validation.json'),'w'); assert(f>=0);
fprintf(f,'%s\n',jsonencode(report)); fclose(f);
fprintf('PASS Stage3 final zero: %d cases (success %d, dwell %d, unaffected %d, no PWM %d, STOP/fault/timeout %d).\n',sum(n),n);

function s=initial(mode,item)
s=struct('mode',mode,'item',item,'state',5,'phase',26,'pwmTick',NaN, ...
    'posts',0,'target',NaN,'recovered',item-1,'finish',NaN,'stopPath',false);
end

function s=step(s,event,now)
if s.stopPath || s.state==6, return; end
if any(strcmp(event,{'stop','owner_fault','owner_timeout','pwm_reject'}))
    s.stopPath=true; return;
end
if s.phase==26
    if strcmp(event,'pwm_ok') && isnan(s.pwmTick), s.pwmTick=now; end
    if strcmp(event,'poll') && ~isnan(s.pwmTick) && tick(now-s.pwmTick)>=100
        % Retain phase 27/28 top/return checks; this fixture is already at top.
        s.recovered=s.recovered+1;
        if s.item<3
            s.phase=17;
        elseif s.mode==1
            s.phase=31; s.posts=1; s.target=0;
        else
            s.state=6;
        end
    end
elseif s.phase==31 && strcmp(event,'dm_done')
    % Current owner transaction reached DONE, distinct from a stale DONE.
    s.phase=32; s.state=6; s.finish=now;
end
end

function t=tick(t)
t=mod(t,2^32);
end
