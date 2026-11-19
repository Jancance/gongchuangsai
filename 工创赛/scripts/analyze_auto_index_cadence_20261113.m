function analyze_auto_index_cadence_20261113
% Actual recorded vision windows, not controller dwell or motion permission.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-13','source-auto-index-observe-20261113-a');
a=readmatrix(fullfile(out,'motion-input.csv'));assert(size(a,1)>1000 && size(a,2)==16);
t=a(:,2);assert(all(diff(t)>0));t=t-t(1);ready=a(:,3)==1;
% Keep exact contiguous sender gates. Do not fill missing/moving frames.
start=find(ready & [true;~ready(1:end-1)]);
stop=find(ready & [~ready(2:end);true]);assert(numel(start)==numel(stop));
duration=t(stop)-t(start);colors=a(start,7:9);
complete=start>1 & stop<size(a,1);usable=complete & duration>=.15;
windows=[t(start),t(stop),duration,colors];
assert(any(usable));
% Aggregate separated gates only as a diagnostic, never as a ready interval.
groups=zeros(numel(start),1);group=0;last=0;
for k=1:numel(start)
 if last==0 || t(start(k))-t(stop(last))>.30 || ~isequal(colors(k,:),colors(last,:)),group=group+1;end
 groups(k)=group;last=k;
end
envelopes=zeros(group,6);
for k=1:group
 ix=find(groups==k);envelopes(k,:)=[t(start(ix(1))),t(stop(ix(end))), ...
 t(stop(ix(end)))-t(start(ix(1))),colors(ix(1),:)];
end
report=struct('engine','local MATLAB','passed',true,'frames',size(a,1), ...
 'recorded_span_s',t(end),'exact_gate_window_count',numel(start), ...
 'complete_gate_windows_at_least_150ms',windows(usable,:), ...
 'complete_gate_duration_s_min_median_max',[min(duration(usable)),median(duration(usable)),max(duration(usable))], ...
 'same_color_gate_envelopes_diagnostic_only',envelopes, ...
 'processing_after_host_read_ms_median_p95_max',[median(a(:,16)),prctile(a(:,16),95),max(a(:,16))], ...
 'host_frame_interval_s_median_p95_max',[median(diff(t)),prctile(diff(t),95),max(diff(t))], ...
 'sensor_exposure_age_verified',false,'controller_dwell_measured',false, ...
 'clearance_time_measured',false,'hardcoded_dwell_seconds',false,'motion_allowed',false);
f=fopen(fullfile(out,'matlab-cadence-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('Actual%d frames over%.3fs; exact continuous eligible windows min/median/max %.3f %.3f %.3fs\n', ...
 size(a,1),t(end),report.complete_gate_duration_s_min_median_max);
fprintf('Host processing median/p95/max %.2f %.2f %.2fms; no controller dwell or safe clearance claim\n', ...
 report.processing_after_host_read_ms_median_p95_max);
end
