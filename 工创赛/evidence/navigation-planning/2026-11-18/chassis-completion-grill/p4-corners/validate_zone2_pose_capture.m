function receipt=validate_zone2_pose_capture(output)
% Single writer and first-run-only diagnostic logger. No navigation inputs.
cap=2048;
period=100;
wrap=2^32;
base=wrap-40;
events=[0 35 99 103 203 301 440 441 500];
source=mod(base+[0 0 0 25 25 200 211 211 460],wrap);
valid=[1 1 1 1 1 1 0 0 1];
phase=[1 1 1 2 3 3 3 3 10];
legs=[0 0 0 0 1 1 1 1 8];
state=initial(cap);
for i=1:numel(events)
    state=step(state,mod(base+events(i),wrap),source(i),valid(i),phase(i),legs(i),...
        i==1,i==numel(events),[1120.37 129.94 -179.996],period);
end
assert(isequal([state.records(1:state.count).elapsed],[0 103 203 440 500]));
assert(isequal([state.records(1:state.count).source],source([1 4 5 7 9])));
assert(isequal([state.records(1:state.count).phase],phase([1 4 5 7 9])));
assert(isequal([state.records(1:state.count).meta],[16 16 17 1 56]));
assert(isequal(state.records(1).pose,[11204 1299 -18000]));
assert(isequal(state.records(4).pose,[0 0 0]));
assert(state.frozen && ~state.saturated && state.count==5);
jitter=state;
state=step(state,82,5,true,1,0,true,false,[0 0 0],period);
assert(isequal(state,jitter));
state=initial(cap);
state=step(state,99,99,true,0,0,false,false,[0 0 0],period);
assert(state.count==0 && ~state.started);
for i=0:cap+2
    tick=mod(base+i*period,wrap);
    state=step(state,tick,tick,true,3,4,i==0,false,[0 0 0],period);
end
assert(state.saturated && state.count==cap-1 && ~state.frozen);
assert(isequal([state.records(1:cap-1).elapsed],(0:cap-2)*period));
before=state.records(1:cap-1);
terminalElapsed=(cap+2)*period+1;
state=step(state,mod(base+terminalElapsed,wrap),0,false,8,4,false,true,[nan 3 2],period);
assert(isequal(before,state.records(1:cap-1)));
assert(state.frozen && state.count==cap && state.records(cap).elapsed==terminalElapsed);
assert(state.records(cap).meta==36 && state.records(cap).source==0);
assert(isequal(state.records(cap).pose,[0 0 0]));
same=initial(cap);
same=step(same,123,100,true,1,0,true,false,[0 0 0],period);
same=step(same,123,100,true,8,0,false,true,[0 0 0],period);
assert(same.count==2 && isequal([same.records(1:2).elapsed],[0 0]));
malformed=initial(cap);
malformed=step(malformed,0,0,true,1,0,true,false,[3276.8 3 0],period);
assert(malformed.records(1).meta==0 && all(malformed.records(1).pose==0));
receipt=struct('version',version,'passed',true,'model_only',true,'hardware_accessed',false,...
    'cadence_ms',period,'capacity',cap,'periodic_capacity',cap-1,'record_bytes',16,...
    'jitter_elapsed_ms',[jitter.records(1:jitter.count).elapsed],...
    'terminal_at_full_elapsed_ms',state.records(cap).elapsed,'uint32_wrap_tested',true,...
    'invalid_and_duplicate_sources_tested',true,'first_run_frozen',true);
if nargin>0
    f=fopen(output,'w');
    assert(f>=0);
    fprintf(f,'%s\n',jsonencode(receipt));
    fclose(f);
end
fprintf('ZONE2_POSE_CAPTURE_PASS %d periodic records, reserved terminal, %d ms cadence\n',cap-1,period);
end

function s=initial(cap)
r=struct('elapsed',0,'source',0,'pose',[0 0 0],'phase',0,'meta',0);
s=struct('started',false,'frozen',false,'saturated',false,'count',0,...
    'start',0,'lastPeriodic',0,'records',repmat(r,1,cap));
end

function s=step(s,now,source,valid,phase,legs,begin,terminal,pose,period)
if s.frozen || (~s.started && ~begin)
    return;
end
first=~s.started;
if first
    s.started=true;
    s.start=now;
end
if ~terminal
    if ~first && mod(now-s.lastPeriodic,2^32)<period
        return;
    end
    if s.count>=numel(s.records)-1
        s.saturated=true;
        return;
    end
    s.lastPeriodic=now;
end
value=round(pose.*[10 10 100]);
valid=valid && all(isfinite(value) & value>=-32768 & value<=32767);
if ~valid
    value=[0 0 0];
end
s.count=s.count+1;
s.records(s.count)=struct('elapsed',mod(now-s.start,2^32),'source',source,'pose',value,...
    'phase',phase,'meta',legs+16*valid+32*terminal);
s.frozen=terminal;
end
