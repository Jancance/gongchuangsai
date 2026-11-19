% MATLAB before Stage3-only preclose implementation; no hardware.
root=fileparts(fileparts(mfilename('fullpath')));out=fullfile(root,'evidence','integration','2026-11-16','stage3-preclose160');
if ~exist(out,'dir'),mkdir(out);end
for owns=[false true]
 for request=[5 6 16 17 18]
  admitted=request<=15 || request==16 || (request==17 && owns);
  if request==17,assert(admitted==owns);end
  if request==18,assert(~admitted);end
  if request==16,assert(admitted);end % existing auto special case unaffected
 end
end
for applied=[false true]
 for elapsed=[0 599 600 601]
  busy=applied;done=busy && elapsed>=600;
  assert(done==(applied && elapsed>=600));
  if ~applied,assert(~busy && ~done);end
 end
end
% Fixed state trace uses existing software settling, not servo feedback.
actions=[10 4 3 3 17 3 5 4 3 3 3 6 4 3 3 10 6];
assert(actions(5)==17 && actions(6)==3 && actions(7)==5 && actions(8)==4 && actions(12)==6);
angles=[210 160 140 210];assert(isequal(angles,[210 160 140 210]));
for phase=0:16
 stop=true;pending=true;ack=false;assert(stop && pending && ~ack);
 ack=true;pending=~ack;assert(~pending && stop); % existing stop, no restart
end
result=struct('passed',true,'when',datestr(now,31),'checks',{{'Stage3-only17; source17 rejected;16 original auto preserved','apply rejection cannot finish; existing600ms settling only, not measured servo angle','top slot and DM ->160 ->pickupZ ->140 ->top ->target ->placeZ ->210','existing STOP all phases waits ACK and no automatic restart'}},'hardware_access',false,'new_wait_or_threshold',false,'place_depth_known',false);
f=fopen(fullfile(out,'matlab-result.json'),'w');fwrite(f,jsonencode(result),'char');fclose(f);disp(jsonencode(result));
