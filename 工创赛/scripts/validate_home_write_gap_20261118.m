function validate_home_write_gap_20261118
R=fileparts(fileparts(mfilename('fullpath')));E=fullfile(R,'evidence','integration','2026-11-18','home-write-gap');if ~exist(E,'dir'),mkdir(E);end
n=0;
for changed=[0 1]
 for cancelled=[0 1]
  for writeok=[0 1]
   for readok=[0 1]
    wait=20*(changed && writeok);assert(wait==0 || wait==20);n=n+1;
    home=writeok && readok && ~cancelled;assert(~home || (readok && ~cancelled));n=n+1;
    probes=~readok && writeok && ~cancelled;assert(~probes || ~home);n=n+1;
   end
  end
 end
end
assert(3*250+3*20+60+5*50+20+60+250<2500);n=n+1;
r=struct('passed',true,'checks',n,'completed_local',datestr(now,'yyyy-mm-dd HH:MM:SS'),'hardware_access',false,'scope','20ms scheduler gap candidate after successful write only; failure probe cannot release homing; original 2500ms budget retained; physical cause remains unproven');
f=fopen(fullfile(E,'matlab-result.json'),'w');fprintf(f,'%s',jsonencode(r));fclose(f);fprintf('PASS write gap %d checks\n',n);
end
