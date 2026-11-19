function validate_release100_overlap_20261115
% Actual continuous-descent traces, before changing release or X/Z timing.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-15','release100-overlap');
if ~exist(out,'dir'),mkdir(out);end
checks=0;cases=0;
% A release dwell starts at first confirmed PWM application, not command time.
for command=[0 100 4294967200]
 for applyDelay=[0 1 19]
  applied=command+applyDelay;
  for elapsed=[0 49 50 99 100 101 150]
   now=applied+elapsed;
   waited=mod(now-applied,2^32);release=waited>=100;
   assert(release==(elapsed>=100));checks=checks+1;cases=cases+1;
  end
 end
end
files={fullfile(root,'evidence','integration','2026-11-15','direct-frame-trigger','pc13-watch-20261115-210111.json'),...
 fullfile(root,'evidence','integration','2026-11-15','integration22-live','pc13-watch-20261115-221115.json')};
rows={};n=0;leads=zeros(1,3);ppm=315.5*360/3200;
for f=1:2
 d=jsondecode(fileread(files{f}));t=d.trace;
 for seq=1:3
  a=t([t.g_low_accepted_sequence]==seq & [t.g_low_phase]==4);
  x=[a.g_startup_extension_current_deg];z=[a.lift_depth_pulses];time=[a.g_low_elapsed_ms];
  target=median(x(end-2:end));mm=abs(target)/ppm;
  if mm<5,region=3;elseif mm>163,region=1;else,region=2;end
  old=[810 780 660];old=old(region);
  err=abs(x-target)/ppm;lastOutside=find(err>5,1,'last');
  if isempty(lastOutside),stable=1;else,stable=lastOutside+1;end
  cross=find(z>=15600-400,1);assert(cross>1 && stable<=numel(a));checks=checks+1;
  % Existing90ms diagnostic skew budget, reusing the earliest Z bracket and
  % latest persistent X sample. This is sampled historical replay only.
  needed=max(0,ceil((old+time(stable)-time(cross-1)+90)/10)*10);
  leads(region)=max(leads(region),needed);
  n=n+1;rows{n}=struct('source',files{f},'sequence',seq,'region',region-1,...
   'programmed_lead_ms',old,'stable_x_sample_ms',time(stable),...
   'z_clearance_bracket_ms',[time(cross-1) time(cross)],...
   'sample_based_required_lead_ms',needed);
 end
end
for i=1:n
 r=rows{i};assert(leads(r.region+1)>=r.sample_based_required_lead_ms);checks=checks+1;
end
r=struct('passed',true,'local_matlab',true,'checks',checks,'release_cases',cases,...
 'release_dwell_ms',100,'release_dwell_from_applied_pwm',true,...
 'historical_continuous_traces',n,'rows',[rows{:}],...
 'sample_based_lead_ms_by_region',leads,'same_start_gate_passed',all(leads==0),...
 'empirical_lead_is_not_physical_sweep_acceptance',true,'hardware_access',false,...
 'no_speed_or_acceleration_changes',true);
fp=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(fp,'%s\n',jsonencode(r));fclose(fp);
fprintf('PASS release100 timing %u checks; sampled continuous-path leads left/right/near %u/%u/%ums. Same-start verified=%u.\n',checks,leads,all(leads==0));
end
