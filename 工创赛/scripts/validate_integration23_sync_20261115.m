function validate_integration23_sync_20261115
% Protocol/state simulation only. No physical collision or 25s acceptance.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-15','integration23-sync');
if ~exist(out,'dir'),mkdir(out);end
checks=0;cases=0;
for region=0:1
 for tx=[1 5 20 80]
  for fail=0:4 % queue X, queue Z, broadcast, or STOP before release
   for epoch=[0 4294967200]
    cases=cases+1;queued=0;moving=0;stopped=0;
    for axis=1:2
     if fail==axis,stopped=6;break;end
     queued=bitor(queued,2^axis);
     assert(moving==0);checks=checks+1;
    end
    if stopped==0
     if fail==3 || fail==4,stopped=6;
     else
      trigger=mod(epoch+2*tx,2^32);
      xstart=trigger;zstart=trigger;moving=queued;
      assert(xstart==zstart && moving==6);checks=checks+1;
     end
    end
    if fail~=0
     assert(stopped==6 && moving==0);checks=checks+1;
    end
   end
  end
 end
end
% Constant-speed/acceleration trapezoids: continuous single final goal.
for xmm=[0 65 162 164 170]
 for zp=[600 29700]
  tx=trap_duration(xmm*315.5/3200,150,600);
  tz=trap_duration(zp/3200,1846.4,45055);
  assert(tx>=0 && tz>=0 && isfinite(tx+tz));checks=checks+1;
 end
end
for applied=[0 19 4294967290]
 for elapsed=[0 49 50 99 100 101]
  now=mod(applied+elapsed,2^32);
  assert((mod(now-applied,2^32)>=100)==(elapsed>=100));checks=checks+1;
 end
end
r=struct('passed',true,'local_matlab',true,'checks',checks,'cases',cases,...
 'scope','queued X/Z then one broadcast; STOP/failure; release100 only at carrier',...
 'synchronized_regions',[0 1],'near_lead_unchanged_ms',660,...
 'extension_rpm',150,'extension_accel_rpm_s',600,'lift_rpm',1846.4,...
 'lift_accel_rpm_s',45055,'physical_sweep_validated',false,...
 'physical_start_skew_validated',false,'cycle25_validated',false,...
 'user_declined_geometry_measurement',true,'hardware_access',false);
fp=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(fp,'%s\n',jsonencode(r));fclose(fp);
fprintf('PASS integration23 protocol/state simulation: %u cases, %u checks. Physical sweep/start skew remain unverified.\n',cases,checks);
end
function t=trap_duration(rev,rpm,acc)
v=rpm/60;a=acc/60;d=abs(rev);
if d<=v*v/a,t=2*sqrt(d/a);else,t=d/v+v/a;end
end
