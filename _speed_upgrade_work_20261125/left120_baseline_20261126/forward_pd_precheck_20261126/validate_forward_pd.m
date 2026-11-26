function validate_forward_pd()
root=fileparts(mfilename('fullpath'));
input=jsondecode(fileread(fullfile(root,'forward_pd_input.json')));
profile=input.profile; gains=[2.75 .25;3.0 .25;3.25 .25];
assert(isequal(size(profile),[25 4]));
assert(all(profile(end,:)==0));
assert(all(round([-1.5 -.5 .5 1.5])==[-2 -1 1 2]));
assert(wrap(179-(-179))==-2 && wrap(-179-179)==2);
results=cell(1,numel(input.runs));
fprintf('Fixed old HWT shadow only: no prediction of new closed-loop vehicle trajectory.\n');
for run=1:numel(input.runs)
    q=input.runs(run); c=q.C; h=q.H;
    assert(size(c,1)==q.END(1) && size(h,1)==q.END(3));
    assert(q.END(4)==0 && q.END(5)==0);
    h=h(bitand(uint32(h(:,5)),uint32(3))==3,:);
    [at,ai]=unique(h(:,1),'last'); [rt,ri]=unique(h(:,2),'last');
    target=q.target_proxy_deg; yaw=wrap(h(ai,3)/100-target); rate=h(ri,4)/100;
    actual=zeros(25,1); times=actual; limits=actual; ackTimes=actual;
    for k=1:25
        gen=q.first_profile_generation+k-1;
        rows=sortrows(c(c(:,1)==gen,:),2); assert(isequal(rows(:,2)',1:4));
        base=profile(k,:); mag=abs(rows(:,3))';
        trim=(mag-base)./[-1 1 -1 1]; assert(all(trim==trim(1)));
        actual(k)=trim(1); times(k)=rows(1,4); limits(k)=min(20,min(base));
        prev=c(c(:,1)==gen-1,:); assert(size(prev,1)==4);
        ackTimes(k)=max(prev(:,6));
    end
    % Sampling decision is not captured. Previous ACK is a reproducible proxy.
    offsets=(-80:5:0)'; counts=zeros(numel(offsets),1);
    for j=1:numel(offsets)
        predicted=shadow(gains(1,:),times+offsets(j),at,yaw,rt,rate,limits);
        counts(j)=sum(predicted==actual);
    end
    [best,bestIndex]=max(counts);
    r=struct('run_id',q.run_id,'input_sha256',q.sha256,'target_proxy_deg',target,...
        'target_is_inferred',true,'profile_batches',25,'baseline_offset_grid_ms',offsets',...
        'baseline_offset_matches',counts','best_offset_ms',offsets(bestIndex),...
        'best_baseline_matches',best,'baseline_total',25);
    fprintf('%s targetProxy %.3f bestBaseline %u/25 at slot%+dms\n',q.run_id,target,best,offsets(bestIndex));
    candidate=cell(1,size(gains,1));
    for g=1:size(gains,1)
        trim=shadow(gains(g,:),ackTimes,at,yaw,rt,rate,limits);
        mixed=profile+trim*[-1 1 -1 1];
        assert(all(mixed(:)>=0 & mixed(:)<=417));
        assert(all(sum(mixed,2)==sum(profile,2)));
        assert(all(abs(trim)<=limits) && all(trim==round(trim)));
        assert(trim(1)==0 && trim(end)==0);
        active=limits>0;
        v=struct('kp',gains(g,1),'kd',gains(g,2),'ack_proxy_trim_rpm',trim',...
            'ack_proxy_baseline_match_count',sum(trim==actual),...
            'actual_trim_rpm',actual','limit_rpm',limits',...
            'at_nonzero_limit_batches',sum(active & abs(trim)>=limits),...
            'at_20rpm_batches',sum(abs(trim)>=20),'mixed_peak_rpm',max(mixed(:)),...
            'wheel_rpm_headroom_to417',417-max(mixed(:)),...
            'changed_vs_baseline_batches',0);
        old=shadow(gains(1,:),ackTimes,at,yaw,rt,rate,limits);
        v.changed_vs_baseline_batches=sum(trim~=old);
        changed=find(trim~=old);
        middle=min(profile,[],2)>=100;
        v.changed_profile_indices=changed';
        v.middle_speed_changed_profile_indices=find((trim~=old)&middle)';
        v.middle_speed_changes=sum((trim~=old)&middle);
        v.delta_trim_rpm=(trim-old)';
        v.sampling_offset_changed_counts=zeros(size(offsets));
        v.sampling_offset_middle_changed_counts=zeros(size(offsets));
        for j=1:numel(offsets)
            t=times+offsets(j);
            reference=shadow(gains(1,:),t,at,yaw,rt,rate,limits);
            alternative=shadow(gains(g,:),t,at,yaw,rt,rate,limits);
            v.sampling_offset_changed_counts(j)=sum(alternative~=reference);
            v.sampling_offset_middle_changed_counts(j)=sum((alternative~=reference)&middle);
        end
        candidate{g}=v;
        fprintf('  KP%.2f KD%.2f ACKproxyMatch%u/25 trim%d..%d peakWheel%u headroom%u limit%u changed%u\n',...
            gains(g,1),gains(g,2),sum(trim==actual),min(trim),max(trim),...
            max(mixed(:)),417-max(mixed(:)),v.at_nonzero_limit_batches,v.changed_vs_baseline_batches);
    end
    r.ack_proxy_baseline_mismatch_indices=find(shadow(gains(1,:),ackTimes,at,yaw,rt,rate,limits)~=actual)';
    r.candidates=[candidate{:}]; results{run}=r;
    outside=c(c(:,1)<q.first_profile_generation | c(:,1)>=q.first_profile_generation+25,:);
    assert(all(outside(:,3)==0));
end
fid=fopen(fullfile(root,'forward_pd_precheck_result.json'),'w'); cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s',jsonencode([results{:}]));
fprintf('PASS: 3 traces x 25 complete profile batches x 3 gains; wrap/round/mix/min-base/start/end/417rpm bounds.\n');
fprintf('Sampling/target uncertainty prevents claiming perfect baseline reproduction or closed-loop improvement. MATLAB %s\n',version);
end

function trim=shadow(gain,t,at,yaw,rt,rate,limit)
assert(min(t)>=min(at) && max(t)<=max(at));
assert(min(t)>=min(rt) && max(t)<=max(rt));
y=interp1(at,yaw,t,'previous'); r=interp1(rt,rate,t,'previous');
correction=-gain(1)*y-gain(2)*r;
trim=round(max(-limit,min(limit,correction)));
end

function v=wrap(v)
v=mod(v+180,360)-180;
end
