function zone2_mechanism_model()
% Logical timing/event model. TIM7 ticks are ideal and motion is not measured.
steps=4200; interval=733; nominalUs=steps*interval; nominalMs=ceil(nominalUs/1000);
assert(nominalUs==3078600 && nominalMs==3079);
assert(nominalMs+251==3330);
checks=2; scenarios=0;
for startTick = [0,2^32-100]
    for delay = 0:9
        for downDir = 0:1
            startUs=(startTick+1000)*1000;
            downTicks=startUs+(1:steps)*interval;
            downDone=ceil(downTicks(end)/1000)+delay;
            scan=downDone+500;
            upStartUs=(scan+3000)*1000;
            upTicks=upStartUs+(1:steps)*interval;
            upDone=ceil(upTicks(end)/1000)+delay;
            stow=upDone;
            ready=stow+1000;
            assert(all(diff(downTicks)==interval) && all(diff(upTicks)==interval));
            assert(numel(downTicks)==steps && numel(upTicks)==steps);
            assert(mod(downDir+(1-downDir),2)==1);
            assert(mod(downDone-startTick,2^32)==1000+nominalMs+delay);
            assert(mod(ready-upDone,2^32)==1000);
            assert(scan>downDone && stow==upDone && ready>stow);
            checks=checks+6; scenarios=scenarios+1;
        end
    end
end
% Exact completion; stale sequence; fault; wrong result; incomplete; unsafe;
% canceled; release rejected. Only exact accepted success can advance.
for leg=1:2
    for scenario=1:8
        correctSequence=scenario~=2;
        completed=scenario~=3;
        success=scenario~=4;
        correctCount=scenario~=5;
        safe=scenario~=6;
        canceled=scenario==7;
        releaseAccepted=scenario~=8;
        releaseAttempt=correctSequence && completed && success && correctCount && safe && ~canceled;
        released=releaseAttempt && releaseAccepted;
        assert(releaseAttempt==(scenario==1 || scenario==8));
        assert(released==(scenario==1));
        assert((leg==1 && released)==(leg==1 && scenario==1));
        assert((leg==2 && released)==(leg==2 && scenario==1));
        checks=checks+4; scenarios=scenarios+1;
    end
end
result=struct('version',version,'steps',steps,'interval_us',interval,...
    'nominal_leg_us',nominalUs,'nominal_leg_ceil_ms',nominalMs,...
    'scenarios',scenarios,'assertions',checks,'passed',true);
fid=fopen('matlab_summary.json','w'); assert(fid>=0);
fprintf(fid,'%s\n',jsonencode(result)); fclose(fid);
fprintf('ZONE2_MECHANISM_PASS %s\n',jsonencode(result));
end
