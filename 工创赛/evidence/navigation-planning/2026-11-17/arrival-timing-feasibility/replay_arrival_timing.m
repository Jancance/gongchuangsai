function replay_arrival_timing
% Estimator feasibility on recorded trajectories, NOT a closed-loop simulation.
out=fileparts(mfilename('fullpath'));diary(fullfile(out,'matlab_diary.log'));
C=jsondecode(fileread(fullfile(out,'contexts.json')));P=jsondecode(fileread(fullfile(out,'parameters.json')));
S=struct('status','RUNNING','matlab_version',version,'hardware_access',false,'firmware_changed',false,'ground_truth_used_for_fitting',false,'closed_loop_simulation',false,'reference_start_offset_s',[-.055 0 .055],'runs',{{}});
try
for n=1:numel(C)
 c=C(n);d=jsondecode(fileread(c.decoded));tr=double(d.header.trigger_ms);ep=c.endpoint;
 switch c.direction
  case 'forward',pr=[1 1 1 1];scale=P.forward_scale;
  case 'backward',pr=[-1 -1 -1 -1];scale=P.forward_scale;
  case 'left',pr=[1 -1 -1 1];scale=1;
  case 'right',pr=[-1 1 1 -1];scale=1;
 end
 T=cell(1,4);X=T;A=T;events=[];
 for j=1:4
  a=d.wheel(strcmp({d.wheel.source},'position') & [d.wheel.valid] & [d.wheel.wheel]==j-1);
  t=(double([a.source_ms]')-tr)/1000;o=(double([a.observed_ms]')-tr)/1000;
  v=double([a.value]').*(1-2*double([a.sign]'));base=ep.(sprintf('start_raw%d',j))*(1-2*ep.(sprintf('start_sign%d',j)));
  v=(v-base)*P.motor_forward_sign(j)*pr(j)*pi*P.wheel_diameter_mm/P.counts_per_rev*scale;
  [t,ix]=unique(t,'first');v=v(ix);o=o(ix);assert(all(diff(t)>0) && all(o>=t));
  T{j}=t;X{j}=v;A{j}=o;events=[events;o]; %#ok<AGROW>
 end
 % No future data in stale or candidate values. Full-source interpolation below
 % is only a retrospective reference for estimator error, NOT ground truth.
 events=unique(events);events=events(events>=0 & events<=c.STOP_s+1e-9);
 profile=readmatrix(c.profile_trace);[~,profile_idx]=unique(profile(:,1),'last');profile=profile(profile_idx,:);assert(all(diff(profile(:,1))>0));trace=[];
 for k=1:numel(events)
  now=events(k);ids=cell(1,4);latest=zeros(1,4);valid=true;
  for j=1:4
   ids{j}=find(A{j}<=now+1e-10);if isempty(ids{j}),valid=false;break;end
   latest(j)=T{j}(ids{j}(end));
  end
  if ~valid,continue;end
  ct=min(latest);old=zeros(1,4);truth=old;
  for j=1:4
   ii=ids{j};if ct<T{j}(ii(1)) || numel(ii)<2 || now>T{j}(end),valid=false;break;end
   old(j)=interp1(T{j}(ii),X{j}(ii),ct,'linear');truth(j)=interp1(T{j},X{j},now,'linear');
  end
  if ~valid,continue;end
  p=mean(old);proxy=mean(truth);pred=zeros(1,3);
  for z=1:3
   offset=S.reference_start_offset_s(z);t1=max(0,min(profile(end,1),now-offset));t0=max(0,min(profile(end,1),ct-offset));
   delta=interp1(profile(:,1),profile(:,2),t1,'linear')-interp1(profile(:,1),profile(:,2),t0,'linear');
   pred(z)=p+delta;
  end
  assert(all(isfinite([p proxy pred])) && all(pred>=p-1e-8));
  trace=[trace;now,ct,1000*(now-ct),p,proxy,pred]; %#ok<AGROW>
 end
 assert(size(trace,1)>30);
 tail=trace(:,5)>=c.target_mm-120 & trace(:,1)<=c.STOP_s+1e-9;assert(sum(tail)>10);
 err=trace(tail,4)-trace(tail,5);e=trace(tail,6:8)-trace(tail,5);
 r=struct('name',c.name,'direction',c.direction,'target_mm',c.target_mm,'samples',size(trace,1),'tail_samples',sum(tail),...
 'stale_tail_bias_mae_maxabs_mm',[mean(err) mean(abs(err)) max(abs(err))],...
 'candidate_tail_bias_mm',mean(e),'candidate_tail_mae_mm',mean(abs(e)),'candidate_tail_maxabs_mm',max(abs(e)),...
 'candidate_tail_min_max_error_mm',[min(e);max(e)],'age_tail_min_median_max_ms',[min(trace(tail,3)) median(trace(tail,3)) max(trace(tail,3))],...
 'actual_STOP_s',c.STOP_s,'frozen_progress_mm',c.frozen_progress_mm,'ground_final_mm_reference_only',c.ground_main_mm,...
 'candidate_crossings',{{}});
 oldidx=find(trace(:,4)>=c.target_mm,1);if isempty(oldidx),r.old_crossing_event_s=[];else,r.old_crossing_event_s=trace(oldidx,1);end
 for z=1:3
  ix=find(trace(:,5+z)>=c.target_mm,1);v=struct('reference_start_offset_s',S.reference_start_offset_s(z),'crossed',~isempty(ix),'event_s',[],'proxy_minus_target_mm',[],'stale_progress_mm',[],'proxy_then_to_actual_STOP_mm',[]);
  if ~isempty(ix)
   v.event_s=trace(ix,1);v.proxy_minus_target_mm=trace(ix,5)-c.target_mm;v.stale_progress_mm=trace(ix,4);v.proxy_then_to_actual_STOP_mm=trace(end,5)-trace(ix,5);
  end
  r.candidate_crossings{z}=v;
 end
 S.runs{n}=r;
 writetable(array2table(trace,'VariableNames',{'receipt_event_s','common_source_s','age_ms','causal_stale_progress_mm','retrospective_body_proxy_mm','candidate_offset_minus55ms_mm','candidate_offset_zero_mm','candidate_offset_plus55ms_mm'}),fullfile(out,[c.name '_replay.csv']));
end
allerr=cellfun(@(r) r.candidate_tail_maxabs_mm(2),S.runs);S.max_candidate_zero_offset_error_mm=max(allerr);
S.status='COMPLETED_WITH_LIMITATIONS';
catch e
 S.status='FAIL';S.error=getReport(e,'extended','hyperlinks','off');writejson(out,S);diary off;rethrow(e);
end
writejson(out,S);disp('ARRIVAL_TIMING_FEASIBILITY_COMPLETED_WITH_LIMITATIONS');diary off;
end
function writejson(out,S)
fid=fopen(fullfile(out,'summary.json'),'w','n','UTF-8');fwrite(fid,jsonencode(S),'char');fclose(fid);
end
