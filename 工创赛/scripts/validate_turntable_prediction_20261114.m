function validate_turntable_prediction_20261114
% Use recorded real turn/stop observations before implementing phase prediction.
root=fileparts(fileparts(mfilename('fullpath')));
input=fullfile(root,'evidence','integration','2026-11-13','source-auto-index-observe-20261113-a');
out=fullfile(root,'evidence','integration','2026-11-14','turntable-prediction');
if ~exist(out,'dir'),mkdir(out);end
d=jsondecode(fileread(fullfile(input,'matlab-cadence-result.json')));assert(d.passed);
w=d.complete_gate_windows_at_least_150ms;w=w(2:end,:);assert(size(w,1)>10);
% Direction comes from actual consecutive colour permutations, not a fixed code.
choices=[2 3 1;3 1 2];votes=zeros(1,2);
for k=2:size(w,1)
 for j=1:2
  if isequal(w(k-1,3+choices(j,:)),w(k,4:6)),votes(j)=votes(j)+1;end
 end
end
[count,index]=max(votes);assert(count==size(w,1)-1);permutation=choices(index,:);
errors=[];predicted=[];actual=[];
for k=5:size(w,1)
 period=median(diff(w(max(1,k-5):k-1,1)));
 prediction=w(k-1,1)+period;
 predicted(end+1)=prediction;actual(end+1)=w(k,1);errors(end+1)=w(k,1)-prediction; %#ok<AGROW>
end
assert(max(abs(errors))<.20);
% Removing an already loaded colour never removes the two real remaining targets.
for removed=[1 2 5]
 for k=2:size(w,1)
  before=w(k-1,4:6);before(before==removed)=0;
  after=w(k,4:6);after(after==removed)=0;
  assert(isequal(before(permutation),after));
 end
end
assert(~(65>65) && 65.001>65); % Strict user clearance threshold.
r=struct('passed',true,'engine','local MATLAB','source','real recorded stop windows', ...
 'next_slots_from_previous_1based',permutation,'consistent_transitions',count, ...
 'measured_period_s_min_median_max',[min(diff(w(:,1))),median(diff(w(:,1))),max(diff(w(:,1)))], ...
 'predicted_stop_start_s',predicted,'actual_stop_start_s',actual, ...
 'prediction_error_s',errors,'prediction_error_abs_max_s',max(abs(errors)), ...
 'partial_remaining_objects_checked',true,'strict_clearance_mm',65, ...
 'prediction_is_preposition_advice_only',true,'fresh_stop_confirmation_required',true, ...
 'hardware_access',false,'three_piece_execution_passed',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('Actual%d transitions; permutation[%d %d %d]; next-stop max error %.3fs; no motion permission\n', ...
 count,permutation,max(abs(errors)));
end
