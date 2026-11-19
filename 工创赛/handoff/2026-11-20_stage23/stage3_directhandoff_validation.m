% Offline motion ordering contract; matching owner DONE represents actual feedback.
here=fileparts(mfilename('fullpath'));
base=jsondecode(fileread(fullfile(here,'stage3_standalone_validation.json')));
cfg=base.config;
v=jsondecode(fileread(fullfile(here,'stage3_serialhandoff_20261120','stage3-vision-13-5.json')));
r=v.ring_row_alignment; c=r.integrated_compensation;
A=r.response_pixel_per_right_forward_ccw(1:2,1:2);
ref=c.reference_center_px(:); target=r.target_center_px(:);
shift=[c.right_mm;c.forward_mm]; calculated=ref+A*shift;
assert(isequal(shift,[5;13]) && ~c.post_alignment_replay);
assert(norm(calculated-target,Inf)<1e-9);
assert(norm(-(A\(ref-target))-shift,Inf)<1e-9);
assert(norm(-(A\(calculated-target)),Inf)<1e-9);
orders=[perms(1:3);1 1 1;2 2 2;3 3 3];
success=0; interrupted=0; noop=0; examples=zeros(size(orders,1),4);
for i=1:size(orders,1)
    first=orders(i,1); last=orders(i,3);
    for e=[0 0.125]
        actual=cfg.x(last)+e;
        s=struct('posted',[],'target',[],'delta',NaN,'fault',false,'pwm',NaN);
        z=advance(s,'poll',1000,actual,cfg,first,last); assert(isempty(z.posted));
        z=advance(s,'pwm_ok',23,actual,cfg,first,last);
        z=advance(z,'poll',122,actual,cfg,first,last); assert(isempty(z.posted));
        p=advance(z,'poll',123,actual,cfg,first,last);
        assert(isequal(p.posted,4) && p.target==cfg.z_place(last)-16000);
        q=advance(p,'old_done',124,actual,cfg,first,last);
        q=advance(q,'dm_done',124,actual,cfg,first,last); assert(isequal(q.posted,4));
        q=advance(q,'z_done',125,actual,cfg,first,last);
        assert(isequal(q.posted,[4 1]) && q.target(end)==cfg.yaw(first));
        x=advance(q,'old_done',126,actual,cfg,first,last);
        x=advance(x,'x_done',126,actual,cfg,first,last); assert(isequal(x.posted,[4 1]));
        x=advance(x,'dm_done',127,actual,cfg,first,last);
        assert(isequal(x.posted,[4 1 2]) && x.target(end)==cfg.x(first));
        assert(x.delta==cfg.x(first)-actual);
        w=advance(x,'poll',128,actual,cfg,first,last);
        w=advance(w,'old_done',128,actual,cfg,first,last);
        assert(isequal(w.posted,[4 1 2]));
        if x.delta==0, noop=noop+1; end
        d=advance(w,'x_done',129,actual,cfg,first,last);
        assert(isequal(d.posted,[4 1 2 4]) && d.target(end)==cfg.z_recover(first));
        for event={'stop','fault','timeout'}
            for pending={z,p,q,x}
                b=advance(pending{1},event{1},129,actual,cfg,first,last);
                for later={'poll','z_done','dm_done','x_done'}
                    b=advance(b,later{1},130,actual,cfg,first,last);
                end
                assert(b.fault && isequal(b.posted,pending{1}.posted));
                interrupted=interrupted+1;
            end
        end
        examples(i,:)=[first last x.target(end) x.delta];
        success=success+1;
    end
end
report=struct('passed',true,'matlab',version,'model_only',true,'physical_acceptance',false, ...
    'success_cases',success,'stop_fault_timeout_cases',interrupted,'noop_cases_with_poll',noop, ...
    'total_cases',success+interrupted,'axis_masks',[4 1 2 4],'orders',orders, ...
    'examples_first_last_x_goal_delta_with_readback_offset',examples,'target_center_px',calculated', ...
    'first_correction_right_forward_mm',(-(A\(ref-target)))','post_alignment_replay',c.post_alignment_replay);
f=fopen(fullfile(here,'stage3_directhandoff_validation.json'),'w'); assert(f>=0);
fprintf(f,'%s\n',jsonencode(report)); fclose(f);
fprintf('PASS Stage3 direct handoff: %d traces; target pixel %.12f, %.12f.\n',report.total_cases,calculated);

function s=advance(s,event,now,actual,cfg,first,last)
if s.fault, return; end
if any(strcmp(event,{'stop','fault','timeout'})), s.fault=true; return; end
axis=0;
switch numel(s.posted)
    case 0
        if strcmp(event,'pwm_ok'), s.pwm=now; end
        if strcmp(event,'poll') && now-s.pwm>=100, axis=4; target=cfg.z_place(last)-16000; end
    case 1
        if strcmp(event,'z_done'), axis=1; target=cfg.yaw(first); end
    case 2
        if strcmp(event,'dm_done'), axis=2; target=cfg.x(first); s.delta=target-actual; end
    case 3
        if strcmp(event,'x_done'), axis=4; target=cfg.z_recover(first); end
end
if axis, s.posted(end+1)=axis; s.target(end+1)=target; end
end
