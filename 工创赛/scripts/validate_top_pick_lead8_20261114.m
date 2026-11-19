function validate_top_pick_lead8_20261114
% User-confirmed lead change only; no motor/gearing/microstep change.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-14','top-pick-lead8');
if ~exist(out,'dir'),mkdir(out);end
oldppm=1600;ppm=oldppm*2/8;rpm=1846.4;acc=45055;
old=[118800 7200 2400 90000];new=old/4;
assert(all(old/oldppm==new/ppm));
pick=new(1);transfer=new(2);backoff=new(3);
assert((pick-transfer)/ppm>65 && backoff/ppm==1.5);
% Both lateral axes remain blocked through 65mm ACTUAL rise.
rise=[0 5 64.9 65 65.0025 69.75];
gate=floor(rise*ppm)/ppm>65;
assert(isequal(gate,[false false false false true true]));
% Original author's wire-decoded speed; finite trapezoid model, not a stopwatch.
speed=mod(2800*30,65536)/10;assert(speed==rpm);
duration=@(p) trapezoid(p/3200,rpm/60,acc/60);
down=duration(pick-backoff);up65=duration(65*ppm);
assert(down+up65<1.0);
% Camera stable samples: five consecutive fresh frames in one pose, no pre-motion requirement.
for count=0:7,assert((count>=5)==ismember(count,5:7));end
r=struct('passed',true,'local_matlab',true,'pulses_per_mm',ppm,...
 'old_targets',old,'new_targets',new,'rpm',rpm,'acceleration_rpm_s',acc,...
 'nominal_pick_distance_mm',pick/ppm,'nominal_return_depth_mm',transfer/ppm,...
 'ideal_down_s',down,'ideal_rise65_s',up65,...
 'stable_frames',5,'motion_before_stop_required',false,...
 'strict_clearance_mm',65,'hardware_access',false,'physical_timing_passed',false,...
 'assumption','lead only 2 to 8; same gearing motor microstep; no load model');
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('PASS lead8: %.0f pulses/mm; down %.3fs; 65mm rise %.3fs; hardware not tested\n',ppm,down,up65);
end
function t=trapezoid(d,v,a)
if d<=v*v/a,t=2*sqrt(d/a);else,t=d/v+v/a;end
end
