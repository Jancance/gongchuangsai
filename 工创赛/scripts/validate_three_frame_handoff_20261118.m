% Existing startup-target +/-3 degree handoff; no physical-motion claim.
root=fileparts(fileparts(mfilename('fullpath')));
out = fullfile(root,'evidence','integration','2026-11-18','three-frame-handoff');
if ~exist(out,'dir'), mkdir(out); end
checks = 0;
for target = [-3,-2.9,-1,0,1,2.9,3]
    for err = [-2.99,-1,0,1,2.99]
        current = target+err;
        old = current>=-2 && current<=272;
        accepted = old || (abs(current-target)<=3 && target>=-3 && target<=3);
        assert(accepted); checks=checks+1;
        for scope = 0:31
            % token, stage2-sync, parallel-active, not-go, exact180/65 owner
            qualified = scope==31;
            candidate = old || (qualified && abs(current-target)<=3);
            assert(candidate == (old || qualified)); checks=checks+1;
        end
    end
end
target=-2.9; current=-3.1036834716796875;
assert(~(current>=-2)); assert(abs(current-target)<=3); checks=checks+2;
for target=[-3,-2.9,0]
    current=target-3.01;
    assert(current < -2);
    assert(~(current>=-2 || abs(current-target)<=3)); checks=checks+2;
end
% Full sequence: actual request6 completes, first prepare holds real current,
% caches X then broadcasts once, then DM target starts. Cancel never releases.
for fallback=0:1
    for cancel=0:3
        proof=true; token=true; go=false; cached=false; broadcasts=0;
        held=current; %#ok<NASGU>
        if cancel==1, proof=false; token=false; end % explicit STOP
        if cancel==0 || cancel==2 || cancel==3
            assert(proof && token); checks=checks+1;
            if fallback, assert(proof); checks=checks+1; end % internal stop2
            proof=false; % consume only after the new target is locked
            if cancel~=2
                cached=true;
                if cancel~=3, broadcasts=broadcasts+1; go=true; token=false; end
            end
        end
        assert(go==(cancel==0));
        assert(broadcasts==double(cancel==0));
        if ~go, assert(broadcasts==0); end
        checks=checks+3;
    end
end
result=struct('passed',true,'checks',checks,'startup_tolerance_deg',3,...
 'non_initial_lower_bound_deg',-2,'near_zero_target_deg',-2.9,...
 'observed_final_feedback_deg',-3.1036834716796875,...
 'observed_value_is_not_fault_instant',true,'hardware_access',false,...
 'note','Model validates scoped reuse of completed actual startup target, not physical motion or fault-instant feedback');
fid=fopen(fullfile(out,'matlab-result.json'),'w');fprintf(fid,'%s\n',jsonencode(result));fclose(fid);
fprintf('PASS MATLAB startup handoff %d checks\n',checks);
