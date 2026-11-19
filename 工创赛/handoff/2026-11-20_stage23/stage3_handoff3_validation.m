% Offline specification model for the final empty-hand Stage3 handoff.
here=fileparts(mfilename('fullpath'));
profiles=jsondecode(fileread(fullfile(here,'stage3_align15_yaw1_cycle_20261120','recovery-profile-candidate.json')));
r=profiles.rings;
z=[r.x1.release_depth_pulses r.x2.release_depth_pulses r.x3.release_depth_pulses];
yaw=[r.x1.yaw_deg r.x2.yaw_deg r.x3.yaw_deg];
handoff=z(2)-8000;
assert(abs(handoff-52795.02378472222)<1e-8 && all(handoff<z));
startOrder=[4 1 2]; orders=perms(1:3); completions=perms(startOrder);
runs=0; stops=0;
for i=1:size(orders,1)
    first=orders(i,1);
    goal=[0 yaw(first) handoff];
    for j=1:size(completions,1)
        [sent,ready,topRef,backoff,retained]=simulate(goal,completions(j,:),0);
        assert(isequal(sent,[4 1 2]) && ready && topRef && ~backoff && isequal(goal,retained));
        runs=runs+1;
        for stopAt=1:4
            [sent,ready,~,~,retained,stopMask]=simulate(goal,completions(j,:),stopAt);
            assert(numel(sent)==stopAt-1 && all(sent==startOrder(1:stopAt-1)) && ~ready && stopMask==7 && isequal(goal,retained));
            stops=stops+1;
        end
    end
    for mode=[0 2]
        assert(~useHandoff(mode,true,false,handoff,z(orders(i,3))));
    end
    assert(~useHandoff(1,true,true,handoff,z(orders(i,3))));
    assert(~useHandoff(1,false,false,handoff,z(orders(i,3))));
    assert(useHandoff(1,true,false,handoff,z(orders(i,3))));
end
report=struct('passed',true,'handoff_depth_pulses',handoff,'start_order',[4 1 2], ...
    'success_cases',runs,'stop_cases',stops,'physical_acceptance',false);
f=fopen(fullfile(here,'stage3_handoff3_validation.json'),'w'); assert(f>=0);
fprintf(f,'%s\n',jsonencode(report)); fclose(f);
fprintf('PASS Stage3 handoff specification: %d ordered cases, %d STOP boundaries; no hardware motion.\n',runs,stops);

function yes=useHandoff(mode,last,req32,target,release)
yes=mode==1 && last && ~req32 && target<release;
end

function [sent,ready,topRef,backoff,retained,stopMask]=simulate(goal,completion,stopAt)
sent=[]; ready=false; topRef=true; backoff=false; retained=goal; stopMask=0;
order=[4 1 2];
for n=1:3
    if stopAt==n, stopMask=7; return; end
    % Lift start precedes the DM target post; extension waits for that post,
    % while completion events remain independent of all start events.
    sent(end+1)=order(n); %#ok<AGROW>
end
if stopAt==4, stopMask=7; return; end
done=0;
for n=1:3
    done=bitor(done,completion(n));
    ready=done==7 && topRef && ~backoff;
    assert(ready==(n==3));
end
end
