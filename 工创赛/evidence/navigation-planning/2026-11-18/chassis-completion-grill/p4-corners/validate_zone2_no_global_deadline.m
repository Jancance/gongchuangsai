function receipt=validate_zone2_no_global_deadline(output)
% Model of the two global time checks and the retained local gates.
globalLimit=180000;
readyLimit=15000;
settleLimit=2000;
elapsed=[0 globalLimit-1 globalLimit globalLimit+1 250000];
checks=0;
for t=elapsed
    for g=0:31
        gates=logical(bitget(uint8(g),1:5)); % active, healthy, map, session, fresh
        candidateCan=all(gates);
        baselineCan=candidateCan && t<globalLimit;
        assert(candidateCan==(g==31));
        assert(baselineCan==(g==31 && t<globalLimit));
        checks=checks+2;
    end
    for local=[14999 15000 15001]
        for phase={'wait_ready','prepare'}
            candidate=step(phase{1},true,true,false,false,local,readyLimit,settleLimit);
            assert(strcmp(candidate,'deadline')==(local>=readyLimit));
            checks=checks+1;
        end
    end
    for local=[1999 2000 2001]
        for stopped=0:1
            candidate=step('section_zero',true,true,true,stopped,local,readyLimit,settleLimit);
            if stopped
                expected='done';
            elseif local>settleLimit
                expected='deadline';
            else
                expected='continue';
            end
            assert(strcmp(candidate,expected));
            checks=checks+1;
        end
    end
    for phase={'track','section_zero'}
        assert(strcmp(step(phase{1},false,true,true,true,0,readyLimit,settleLimit),'health'));
        assert(strcmp(step(phase{1},true,false,true,true,0,readyLimit,settleLimit),'source'));
        checks=checks+2;
    end
    candidate=step('section_zero',true,true,true,true,0,readyLimit,settleLimit);
    baseline=candidate;
    if t>=globalLimit, baseline='deadline'; end
    assert(strcmp(candidate,'done'));
    assert(strcmp(baseline,'done')==(t<globalLimit));
    checks=checks+2;
end
receipt=struct('version',version,'passed',true,'assertions',checks,'model_only',true,...
    'baseline_global_limit_ms',globalLimit,'candidate_global_limit',false,...
    'elapsed_ms',elapsed,'local_ready_prepare_ms',readyLimit,'local_settle_ms',settleLimit,...
    'late_finish_requires_stop_evidence',true,'hardware_accessed',false);
if nargin>0
    fid=fopen(output,'w');
    assert(fid>=0);
    fprintf(fid,'%s\n',jsonencode(receipt));
    fclose(fid);
end
fprintf('ZONE2_NO_GLOBAL_DEADLINE_PASS %d assertions\n',checks);
end

function state=step(phase,healthy,fresh,terminal,stopped,relative,readyLimit,settleLimit)
state='continue';
if ~healthy
    state='health';
elseif strcmp(phase,'wait_ready') || strcmp(phase,'prepare')
    if relative>=readyLimit, state='deadline'; end
elseif ~fresh
    state='source';
elseif strcmp(phase,'section_zero')
    if terminal && stopped
        state='done';
    elseif relative>settleLimit
        state='deadline';
    end
end
end
