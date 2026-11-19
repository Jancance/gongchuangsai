function validate_stage3_full_cycle_20261119
root=fileparts(fileparts(mfilename('fullpath')));
d=jsondecode(fileread(fullfile(root,'calibration','stage3_roughing_ring_teach_20261118.json')));
n=fieldnames(d.ring_points);
x=cellfun(@(k)d.ring_points.(k).extension_mm,n).'; z=cellfun(@(k)d.ring_points.(k).lift_target_pulses,n).'; a=cellfun(@(k)d.ring_points.(k).dm_yaw_deg,n).';

assert(isequal(x,[57.5 2.5 60.5]) && isequal(z,[59400 61400 60600]));
assert(all(a>=0 & a<=270));
orders=perms(1:3); checks=0;
for i=1:6
 for j=1:6
  slot=orders(i,:); ring=orders(j,:); onRing=zeros(1,3); original=slot;
  for k=1:3
   s=find(slot==k); r=ring(k); assert(numel(s)==1 && ~onRing(r));
   % Independent completion delays. One broadcast starts both after yaw DONE.
   for tx=[1 10 100]
    for tz=[1 10 100]
     tYaw=2; tXStart=tYaw+1; tZStart=tXStart;
     tRelease=max(tXStart+tx,tZStart+tz)+1;
     assert(tYaw<tXStart && tXStart==tZStart && tRelease>tZStart+tz && tRelease>tXStart+tx);
     xx=x(r)*min((0:100)/tx,1); zz=60206*min((0:100)/tz,1);
     assert(all(xx>=0 & xx<=x(r)) && all(zz>=0 & zz<=60206));
     checks=checks+1;
    end
   end
   onRing(r)=slot(s); slot(s)=0;
  end
  assert(all(slot==0));
  for k=1:3
   r=ring(k); s=find(original==k);
   assert(onRing(r)==k && z(r)>=3200 && z(r)<=82000);
   tGrip=1; tTop=2; tHome=3; tYawToTray=4;
   assert(tGrip<tTop && tTop<tHome && tHome<tYawToTray && slot(s)==0);
   slot(s)=onRing(r); onRing(r)=0;
  end
  assert(isequal(slot,original) && all(onRing==0));
 end
end
out=fullfile(root,'evidence','integration','2026-11-18','stage3-resume','full-cycle-matlab.json');
fid=fopen(out,'w'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(struct('passed',true,'checks',checks,'physical_clearance_verified',false,'matlab_version',version)));
fprintf('PASS %d task order and two-axis timing cases\n',checks);
end

