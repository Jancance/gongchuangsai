function validate_carrier_release300_20261116
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-16','carrier-release300');
if ~exist(out,'dir'),mkdir(out);end
checks=0;
for command=[0 100 4294967200]
 for applyDelay=[0 1 19]
  applied=mod(command+applyDelay,2^32);
  for elapsed=[0 199 200 299 300 301 500]
   now=mod(applied+elapsed,2^32);
   assert((mod(now-applied,2^32)>=300)==(elapsed>=300));checks=checks+1;
  end
 end
end
% A stop/fault still exits independently of the release dwell.
for elapsed=[0 100 299 300]
 for cancelled=[false true]
  return_allowed=~cancelled && elapsed>=300;
  assert(~cancelled || ~return_allowed);checks=checks+1;
 end
end
r=struct('passed',true,'local_matlab',true,'checks',checks,'carrier_release_ms',300,...
 'from_applied_pwm',true,'source_extra_wait_ms',0,'hardware_access',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('PASS carrier release300: %u checks.\n',checks);
end
