here=fileparts(mfilename('fullpath'));
scale=[10 100]; originLimit=[17.75 22.5];
arrivalLimit=[5*315.5*360/3200 22.5]; home=[0 68.7];
incident=jsondecode(fileread(fullfile(here,'anticipatory-cached-origin-decoded.json')));
assert(strcmp(incident(1).axis,'X') && strcmp(incident(2).axis,'Z'));
goal=[incident.goal]; before=[incident.before]; expected=[incident.current];
rel=zeros(1,2);
for i=1:2
    hex=incident(i).frame_hex;
    assert(strcmp(hex(1:2),'fd') && strcmp(hex(25:28),'0201'));
    rel(i)=(1-2*hex2dec(hex(3:4)))*hex2dec(hex(17:24))/scale(i);
end
assert(permit(expected,rel,goal,home,originLimit,arrivalLimit,[0 0],true,true,false));
assert(permit(before,rel,goal,home,originLimit,arrivalLimit,[0 0],true,true,false));
tested=2;
for sign=[-1 1]
    for axis=1:2
        sample=home; sample(axis)=home(axis)+sign*originLimit(axis);
        assert(permit(sample,goal-sample,goal,home,originLimit,arrivalLimit,[0 0],true,true,false));
        sample(axis)=sample(axis)+sign/scale(axis);
        assert(~permit(sample,goal-sample,goal,home,originLimit,arrivalLimit,[0 0],true,true,false));
        endpoint=goal-home; endpoint(axis)=endpoint(axis)+sign*arrivalLimit(axis);
        assert(permit(home,endpoint,goal,home,originLimit,arrivalLimit,[0 0],true,true,false));
        endpoint(axis)=endpoint(axis)+sign/scale(axis);
        assert(~permit(home,endpoint,goal,home,originLimit,arrivalLimit,[0 0],true,true,false));
        tested=tested+4;
    end
end
assert(~permit(expected,rel,goal,home,originLimit,arrivalLimit,[1 0],true,true,false));
assert(~permit(expected,rel,goal,home,originLimit,arrivalLimit,[0 1],true,true,false));
assert(~permit(expected,rel,goal,home,originLimit,arrivalLimit,[0 0],false,true,false));
assert(~permit(expected,rel,goal,home,originLimit,arrivalLimit,[0 0],true,false,false));
assert(~permit(expected,rel,goal,home,originLimit,arrivalLimit,[0 0],true,true,true));
zeroGoal=[0 goal(2)]; zeroLimit=[originLimit(1) arrivalLimit(2)];
assert(permit(home,[17.7 goal(2)-home(2)],zeroGoal,home,originLimit,zeroLimit,[0 0],true,true,false));
assert(~permit(home,[17.8 goal(2)-home(2)],zeroGoal,home,originLimit,zeroLimit,[0 0],true,true,false));
report=struct('passed',true,'cases',tested+7,'incident_before_deg',before, ...
    'incident_goal_deg',goal,'incident_recheck_deg',expected,'incident_relative_deg',rel, ...
    'incident_endpoint_error_deg',expected+rel-goal,'physical_acceptance',false);
f=fopen(fullfile(here,'stage3_origin_recheck_fix_validation.json'),'w');assert(f>=0);
fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('PASS origin recheck: %d window, recorded-position, health, velocity, freshness and STOP checks.\n',tested+7);

function allow=permit(current,relative,goal,home,originLimit,arrivalLimit,speed,healthy,fresh,stop)
allow=~stop && healthy && fresh && all(speed==0) && ...
    all(abs(current-home)<=originLimit+1e-10) && ...
    all(abs(current+relative-goal)<=arrivalLimit+1e-10);
end
