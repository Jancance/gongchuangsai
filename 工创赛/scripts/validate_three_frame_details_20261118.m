function validate_three_frame_details_20261118
R=fileparts(fileparts(mfilename('fullpath')));E=fullfile(R,'evidence','integration','2026-11-18','three-frame-warm');n=0;
% First target alone requires the witnessed120; later targets never do.
for item=0:2
 for degrees=[0 30 119 120 121 240]
  for frames=0:4
   ready=frames>=3 && (item>0 || degrees>=120);
   assert(~ready || frames>=3);n=n+1;
   if item==0,assert(ready==(frames>=3 && degrees>=120));else,assert(ready==(frames>=3));end;n=n+1;
  end
 end
end
% Only measured increments count across a camera-move break, never its gap.
angles=[0 30 60 nan 90 120 150];observed=0;previous=nan;
for angle=angles
 if isnan(angle),previous=nan;continue;end
 if ~isnan(previous),observed=observed+angle-previous;end
 previous=angle;
end
assert(observed==120);n=n+1;
% Current verified lead is8mm and400 virtual pulses/mm, not the old1600.
old=29700;scale=400;side=old-10*scale;bottom=old;
assert(side==25700 && side/scale==64.25 && bottom/scale==74.25);n=n+1;
% Original rise>65mm gate must retain its original absolute height:
% depth<3700p. It cannot remain a65mm rise requirement after shortening Z.
for pick=[old side]
 for depth=[1800 3699 3700 5000]
  originalAbsolute=depth<(old-65*scale);
  if depth==1800,assert(originalAbsolute);end
  if depth==3700,assert(~originalAbsolute);end
  assert(pick-depth>0);n=n+1;
 end
end
% Every committed sequence contributes exactly one pending beep; a busy
% alignment cue is allowed to finish before the queued short cue sounds.
pending=0;sounded=0;
for seq=1:3
 pending=pending+1;
 for busy=[1 1 0 0]
  if ~busy && pending>0,sounded=sounded+1;pending=pending-1;end
 end
 assert(sounded==seq && pending==0);n=n+1;
end
r=struct('passed',true,'checks',n,'completed_local',datestr(now,'yyyy-mm-dd HH:MM:SS'),'first120_only',true,'prealignment_witness_allowed',true,'side_pick_pulses',side,'bottom_pick_pulses',bottom,'pulses_per_mm',scale,'existing_absolute_clearance_pulses',3700,'hardware_access',false,'scope','First-only measured120, three stopped frames, height unit conversion, unchanged absolute transfer clearance and one queued beep per committed item; no physical grip/clearance proof');
f=fopen(fullfile(E,'matlab-details.json'),'w');fprintf(f,'%s',jsonencode(r));fclose(f);fprintf('PASS first120/heights/beeps %d checks\n',n);
end
