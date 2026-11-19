function estimate_extension_lead_cycle_20261115
% Conditional estimate from old actual timestamps; no new physical latency.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-15','extension-lead-pick');
cfg=jsondecode(fileread(fullfile(out,'timing-config.json')));
old=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-15','integration20-yaw165-window10','committed-20261115-154633.json')));
contact=zeros(1,3);piece=zeros(1,3);checks=0;results={};
% Piece order is right,near,left; protocol region order is left,right,near.
regions=[2 3 1];
for k=1:3
 groups=old.pieces(k).motion_groups;region=regions(k);
 approach=groups([groups.phase]==2);descent=groups([groups.phase]==4);
 turn=groups([groups.phase]==1);back=groups([groups.phase]==5);
 extra=groups([groups.phase]==11);transfer=groups([groups.phase]==12);
 returnGroup=groups([groups.phase]==7);
 yaw=0;if ~isempty(turn),yaw=turn.done_ms-turn.started_ms;end
 % The recorded second half takes ~0.7s including settling. Use twice that
 % as the whole-descent estimate; this is an extrapolation, not a new test.
 fullZ=2*(descent.done_ms-descent.started_ms);
 lead=cfg.extension_lead_ms_by_region(region);
 contact(region)=yaw+approach.started_ms-approach.started_ms+lead+fullZ;
 % Add existing pre-start/transition overhead conservatively per new group.
 contact(region)=contact(region)+134+73;
 extraTime=0;if ~isempty(extra),extraTime=extra.done_ms-extra.started_ms;end
 returnTime=back.done_ms-back.started_ms;
 % Full retract duration estimated by summing old two-part X times, retaining
 % their measured settle overhead rather than claiming a hardware speed gain.
 returnTime=returnTime+extraTime;
 piece(region)=contact(region)+returnTime+(transfer.done_ms-transfer.started_ms)+...
  (returnGroup.done_ms-returnGroup.started_ms)+700+50+4*73;
 assert(contact(region)>0 && piece(region)>contact(region));checks=checks+2;
 results{region}=struct('region',region-1,'lead_to_grip_estimate_ms',contact(region),...
  'piece_duration_estimate_ms',piece(region),'historical_fullZ_extrapolation_ms',fullZ);
end
% Prior actual disc speed ~0.36rad/s =>17.45s/revolution. Without adding
% a new revolution-learning stage, allow0..one revolution per piece for phase.
period=2*pi/.36*1000;mechanical=sum(piece);estimate=[mechanical mechanical+3*period];
config=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-15','source-rim-reacquire','fast-observation','orbit-config.json')));
config.lead_ms_by_region=round(contact);config.source=...
 'Historical timing extrapolation for unchanged-speed extension-first candidate; new physical contact timestamp unmeasured';
f=fopen(fullfile(out,'trial-orbit-config.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(config));fclose(f);
report=struct('local_matlab',true,'checks',checks,'estimate_only',true,...
 'physical_contact_timing_verified',false,'results',[results{:}],...
 'assumed_disc_period_ms',period,'three_piece_after_alignment_estimate_ms',estimate,...
 'chassis_alignment_duration_not_bounded_by_this_estimate',true,...
 'no_full_revolution_learning_required',true,'hardware_access',false);
f=fopen(fullfile(out,'cycle-estimate.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('CONDITIONAL estimate: three pieces after alignment %.1f..%.1fs; alignment extra, not physical timing acceptance.\n',estimate/1000);
end
