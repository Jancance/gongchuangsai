% Offline Begin/Poll/Stop contract; motion-owner events are independent inputs.
here=fileparts(mfilename('fullpath'));
b=jsondecode(fileread(fullfile(here,'stage3_standalone_validation.json')));
d=jsondecode(fileread(fullfile(here,'stage3_directhandoff_validation.json')));
cfg=b.config; assert(d.passed && isequal(d.axis_masks(:)',[4 1 2 4]));
assert(cfg.align_mm==2 && cfg.align_deg==1);
orders=perms(1:3); n=zeros(1,3);
for module=1:3 % Source / Rough / Stack
    for row=1:6
        t=struct('run',101,'batch',1,'colors',[5 2 1],'rings',orders(row,:), ...
            'slots',[2 1 5]);
        s=initial(); s.vehicleStarted=true; % PC13 belongs to the vehicle only.
        assert(s.status==0 && s.sequence==0);
        for invocation=1:2
            t.run=100+invocation; t.batch=invocation;
            [s,ok]=begin(s,module,t); assert(ok && poll(s,t.run)==1);
            [same,ok]=begin(s,module,t); assert(~ok && isequal(same,s));
            old=event(s,'done',t.run,s.sequence-1,0);
            wrong=event(s,'done',t.run+99,s.sequence,0);
            assert(isequal(s,old) && isequal(s,wrong) && poll(s,t.run+99)==0);
            assert(isequal(s.locked.colors,t.colors));
            if module==1, assert(~isequal(s.locked.colors,[2 1 5]) && ~s.rescan); end
            switched=t; switched.colors=[1 5 2];
            [same,ok]=begin(s,module,switched); assert(~ok && isequal(same.locked,t));
            for j=1:3
                m=t.rings(j);
                slot=find(t.slots==t.colors(j));
                s=event(s,'piece',t.run,s.sequence,slot);
                assert(s.count==j && poll(s,t.run)==1);
                if module==3
                    goal=cfg.z_place(m)-24000;
                    assert(s.locked.rings(j)==m && s.locked.colors(j)==t.slots(slot));
                    assert(abs(cfg.z_place(m)-goal-24000)<1e-8 && goal>0);
                    assert(abs(s.stackDepth(m)-goal)<1e-8);
                end
            end
            if module==3
                assert(s.recovered==0 && poll(s,t.run)==1);
                for milestone={'safe_top','x_zero','dm_zero'}
                    s=event(s,'done',t.run,s.sequence,0); assert(poll(s,t.run)==1);
                    if strcmp(milestone{1},'safe_top')
                        s=event(s,'x_zero',t.run,s.sequence,0); assert(s.tail==0);
                    end
                    if strcmp(milestone{1},'x_zero')
                        s=event(s,'dm_zero',t.run,s.sequence,0); assert(s.tail==1);
                    end
                    s=event(s,milestone{1},t.run,s.sequence,0);
                end
                assert(s.tail==3 && s.recovered==0);
            elseif module==2
                for j=1:3, s=event(s,'recover',t.run,s.sequence,0); end
                assert(s.recovered==3 && poll(s,t.run)==1);
                s=event(s,'dm_zero',t.run,s.sequence,0);
            else
                s=event(s,'done',t.run,s.sequence,0);
            end
            assert(poll(s,t.run)==2 && s.publications==invocation);
            s=event(s,'done',t.run,s.sequence,0);
            assert(s.publications==invocation);
            n(1)=n(1)+1;
        end
        [s,ok]=begin(initial(),module,t); assert(ok);
        s=event(s,'stop',t.run,s.sequence,0);
        s=event(s,'done',t.run,s.sequence,0); assert(poll(s,t.run)==3 && ~s.publications);
        [same,ok]=begin(s,module,t); assert(~ok && isequal(same,s));
        s=event(s,'stop_ack',t.run-1,s.sequence,0); assert(poll(s,t.run)==3);
        s=event(s,'stop_ack',t.run,s.sequence,0); assert(poll(s,t.run)==4);
        [same,ok]=begin(s,module,t); assert(~ok && isequal(same,s));
        n(2)=n(2)+1;
        [s,ok]=begin(initial(),module,t); assert(ok);
        s=event(s,'fault',t.run,s.sequence,0);
        s=event(s,'done',t.run,s.sequence,0); assert(poll(s,t.run)==5 && ~s.publications);
        [same,ok]=begin(s,module,t); assert(~ok && isequal(same,s));
        n(3)=n(3)+1;
    end
end
report=struct('passed',true,'matlab',version,'model_only',true,'physical_acceptance',false, ...
    'source_rough_stack_success_cases',n(1),'stop_cases',n(2),'fault_cases',n(3), ...
    'total_cases',sum(n),'stack_offset_pulses',24000,'stack_release_pulses',cfg.z_place-24000, ...
    'ring_orders',orders,'directhandoff_cases_inherited',d.total_cases,'dwell_ms_inherited',100);
f=fopen(fullfile(here,'stage3_modules_validation.json'),'w'); assert(f>=0);
fprintf(f,'%s\n',jsonencode(report)); fclose(f);
fprintf('PASS Stage3 Source/Rough/Stack modules: %d cases across six mappings and two batches.\n',sum(n));

function s=initial()
s=struct('vehicleStarted',false,'status',0,'sequence',0,'locked',[], ...
    'publications',0,'module',0,'count',0,'recovered',0,'tail',0,'rescan',false,'stackDepth',[]);
end
function [s,ok]=begin(s,module,t)
ok=any(s.status==[0 2]) && any(t.batch==[1 2]) && t.run~=0 && ...
    isequal(sort(t.colors),sort(t.slots)) && isequal(sort(t.rings),1:3);
if ~ok, return; end
cfg=jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')),'stage3_standalone_validation.json')));
s.sequence=s.sequence+1; s.locked=t; s.module=module; s.status=1;
s.count=0; s.recovered=0; s.tail=0;
if module==3, s.stackDepth=cfg.config.z_place-24000; end
end
function p=poll(s,run)
p=0;
if s.sequence && s.locked.run==run, p=s.status; end
end
function s=event(s,name,run,seq,slot)
if ~s.sequence || s.locked.run~=run || s.sequence~=seq, return; end
if s.status==3 && strcmp(name,'stop_ack'), s.status=4; return; end
if s.status~=1, return; end
done=false;
if strcmp(name,'stop'), s.status=3;
elseif strcmp(name,'fault'), s.status=5;
elseif strcmp(name,'piece') && s.count<3 && s.locked.slots(slot)==s.locked.colors(s.count+1)
    s.count=s.count+1;
elseif s.count==3
    if s.module==1, done=strcmp(name,'done');
    elseif s.module==2
        if strcmp(name,'recover') && s.recovered<3, s.recovered=s.recovered+1; end
        done=s.recovered==3 && strcmp(name,'dm_zero');
    elseif s.module==3
        tail={'safe_top','x_zero','dm_zero'};
        if strcmp(name,tail{s.tail+1}), s.tail=s.tail+1; end
        done=s.tail==3;
    end
end
if done, s.status=2; s.publications=s.publications+1; end
end
