function receipt=validate_zone2_route_evidence(output)
% The observer supplies accepted transactions. This models their recording.
base=2^32-30;
failedPhase=7;
donePhase=10;
pathEnds=[0 2; 2 14; 14 10; 10 22; 22 14; 14 10; 10 22; 22 0];
ids={2,[12 14],10,[12 22],[12 14],10,[12 22],[2 0]};
wire=[65535 0 1 2 3 1 2 4];
sources=[1 2 3 4 5 3 4 8];
map=uint32([32 7 3 9 65537 12 2 1 2]);
success=event(initial(base),'grid',0,0,map);
assert(success.grid_valid && isequal(success.grid,map));
assert(isequal(event(success,'grid',0,1,uint32(1:9)),success));
for leg=1:8
    t=35+200*(leg-1);
    p=makePath(wire(leg),pathEnds(leg,:),ids{leg});
    success=event(success,'path',leg,t,[sources(leg) p]);
    recorded=success;
    assert(isequal(event(success,'path',leg,t+1,[sources(leg) p]),recorded));
    assert(isequal(event(success,'arrive',leg,t+2,[leg 0]),recorded));
    success=event(success,'begin',leg,t+5,leg);
    started=success;
    assert(isequal(event(success,'begin',leg,t+6,leg+1),started));
    assert(isequal(event(success,'arrive',leg,t+6,[leg+1 0]),started));
    success=event(success,'arrive',leg,t+101,[leg tick(base,t+88)]);
    assert(isequal(event(success,'arrive',leg,t+102,[leg 0]),success));
end
success=event(success,'freeze',0,1536,[0 0 donePhase]);
assert(success.state==2 && success.final_phase==donePhase && ...
    success.final_elapsed_ms==1536 && success.completed_legs==8);
assert(all([success.legs.flags]==7) && isequal([success.legs.session],1:8));
assert(isequal([success.legs.source_leg],sources));
assert(isequal([success.legs.observed_elapsed_ms],35:200:1435));
assert(isequal([success.legs.begin_elapsed_ms],40:200:1440));
assert(isequal([success.legs.arrive_elapsed_ms],136:200:1536));
assert(isequal([success.legs.stop_source_ms],tick(base,123:200:1523)));
assert(isequal([success.legs.seq],uint16(wire)));
assert(isequal(pathBytes(success.legs(3)),pathBytes(success.legs(6))));
assert(isequal(pathBytes(success.legs(4)),pathBytes(success.legs(7))));
assert(isequal(success,event(success,'freeze',0,1600,[3 8 failedPhase])));

bad=event(initial(base),'grid',0,1,map);
bad=event(bad,'path',1,20,[1 makePath(65535,pathEnds(1,:),ids{1})]);
bad=event(bad,'begin',1,25,1);
bad=event(bad,'arrive',1,30,[1 0]); % Actual source time is zero at uint32 wrap.
bad=event(bad,'path',2,31,[2 makePath(0,pathEnds(2,:),[4 14])]);
bad=event(bad,'freeze',0,31,[3 8 failedPhase]); % Schedule rejects occupied cell after receipt.
assert(bad.state==2 && bad.fault==3 && bad.path_error==8 && bad.final_phase==failedPhase);
assert(bad.final_elapsed_ms==31 && bad.completed_legs==1);
assert(isequal([bad.legs.flags],[7 1 0 0 0 0 0 0]));
assert(bad.legs(1).arrive_elapsed_ms==30 && bad.legs(1).stop_source_ms==0);
assert(bad.legs(2).seq==0 && bad.legs(2).count==2 && ...
    isequal(bad.legs(2).ids(1:2),uint8([4 14])));
assert(bad.legs(2).session==0 && bad.legs(2).arrive_elapsed_ms==0);
assert(isequal(event(bad,'begin',2,50,2),bad));

blank=initial(0);
missingMap=event(blank,'freeze',0,0,[2 0 failedPhase]);
assert(~missingMap.grid_valid && all(missingMap.grid==0) && missingMap.final_elapsed_ms==0);
assert(isequal(missingMap.legs,blank.legs) && ~missingMap.completed_legs);
emptyGrid=map;
emptyGrid(1)=0;
missingPath=event(blank,'grid',0,0,emptyGrid);
missingPath=event(missingPath,'freeze',0,3000,[3 4 failedPhase]);
assert(missingPath.grid_valid && missingPath.grid(1)==0);
assert(isequal(missingPath.legs,blank.legs) && ~missingPath.completed_legs);

poseReceipt=validate_zone2_pose_capture();
full=event(blank,'grid',0,0,map);
full=event(full,'path',1,0,[1 makePath(0,pathEnds(1,:),[3:24 1 2])]);
assert(full.legs(1).flags==1 && full.legs(1).observed_elapsed_ms==0);
assert(full.legs(1).seq==0 && full.legs(1).count==24 && full.legs(1).ids(24)==2);
full=event(full,'begin',1,204800,1);
full=event(full,'arrive',1,205001,[1 204998]);
full=event(full,'freeze',0,205001,[3 4 failedPhase]);
assert(full.completed_legs==1 && full.legs(1).flags==7);
assert(full.legs(1).arrive_elapsed_ms==205001 && full.legs(1).stop_source_ms==204998);
assert(poseReceipt.passed && full.final_elapsed_ms==poseReceipt.terminal_at_full_elapsed_ms);

headerBytes=14*4+4;
legBytes=5*4+2+6+24;
assert(headerBytes+8*legBytes==476);
receipt=struct('version',version,'passed',true,'model_only',true,'hardware_accessed',false,...
    'base_head','5ef02305','header_bytes',headerBytes,'leg_bytes',legBytes,'route_bytes',476,...
    'occupied_mask',success.grid(1),'live_paths',sum(sources==1:8),'cache_paths',sum(sources~=1:8),...
    'complete_legs',success.completed_legs,'rejected_flags',[bad.legs.flags],...
    'rejected_path_ids',bad.legs(2).ids(1:2),'rejected_stop_count',bad.completed_legs,...
    'sequence_uint16_wrap_and_zero',true,'tick_uint32_wrap_and_zero',true,...
    'missing_map_empty_map_missing_path_tested',true,'cache_wire_matches',true,...
    'first_terminal_frozen',true,'duplicate_path_begin_arrive_tested',true,...
    'route_retained_after_pose_capacity_time_ms',full.final_elapsed_ms);
if nargin>0
    f=fopen(output,'w');
    assert(f>=0);
    fprintf(f,'%s\n',jsonencode(receipt));
    fclose(f);
end
fprintf('ZONE2_ROUTE_EVIDENCE_PASS %d bytes, 6 live paths, 2 cache uses, %d/%d stops\n',...
    receipt.route_bytes,success.completed_legs,bad.completed_legs);
end

function s=initial(base)
r=struct('observed_elapsed_ms',0,'begin_elapsed_ms',0,'arrive_elapsed_ms',0,...
    'stop_source_ms',0,'session',0,'seq',uint16(0),'status',uint8(0),'start',uint8(0),...
    'goal',uint8(0),'count',uint8(0),'source_leg',0,'flags',0,'ids',zeros(1,24,'uint8'));
s=struct('state',1,'fault',0,'path_error',0,'run_start_ms',base,'final_elapsed_ms',0,...
    'grid_valid',false,'grid',zeros(1,9,'uint32'),'completed_legs',0,...
    'final_phase',0,'legs',repmat(r,1,8));
end

function s=event(s,kind,i,t,value)
if s.state~=1
    return;
end
elapsed=mod(tick(s.run_start_ms,t)-s.run_start_ms,2^32);
switch kind
    case 'grid'
        if ~s.grid_valid
            s.grid=value;
            s.grid_valid=true;
        end
    case 'path'
        if ~s.grid_valid || s.legs(i).flags~=0
            return;
        end
        r=s.legs(i);
        r.observed_elapsed_ms=elapsed;
        r.source_leg=value(1);
        if r.source_leg~=i
            assert(isequal(pathBytes(s.legs(r.source_leg)),value(2:end)));
        end
        r.seq=uint16(value(2));
        r.status=uint8(value(3));
        r.start=uint8(value(4));
        r.goal=uint8(value(5));
        r.count=uint8(value(6));
        r.ids=uint8(value(7:end));
        r.flags=1;
        s.legs(i)=r;
    case 'begin'
        if s.legs(i).flags==1
            s.legs(i).begin_elapsed_ms=elapsed;
            s.legs(i).session=value;
            s.legs(i).flags=3;
        end
    case 'arrive'
        if s.legs(i).flags==3 && s.legs(i).session==value(1)
            s.legs(i).arrive_elapsed_ms=elapsed;
            s.legs(i).stop_source_ms=value(2);
            s.legs(i).flags=7;
            s.completed_legs=s.completed_legs+1;
        end
    case 'freeze'
        s.fault=value(1);
        s.path_error=value(2);
        s.final_phase=value(3);
        s.final_elapsed_ms=elapsed;
        s.state=2;
end
end

function x=tick(base,elapsed)
x=mod(base+elapsed,2^32);
end

function p=makePath(seq,ends,ids)
p=[seq 0 ends numel(ids) ids zeros(1,24-numel(ids))];
end

function p=pathBytes(r)
p=[double(r.seq) double([r.status r.start r.goal r.count r.ids])];
end
