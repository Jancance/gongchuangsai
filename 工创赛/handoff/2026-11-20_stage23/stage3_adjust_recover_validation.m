% Offline discrete-event specification model; no firmware or hardware.
root=['D:/Codex/' char([24037 21019 36187]) '/output/stage3_offline_13_5_20261119'];
m=jsondecode(fileread(fullfile(root,'stage3-offline-compatible.json')));
p=jsondecode(fileread(fullfile(root,'placement-profile.json')));
assert(m.build_passed && ~m.physical_acceptance);
c=adjust(p); assert(isequal(c,adjust(p)));
z=[61802.22274305556 60795.02378472222 63002.22274305556];
rz=[61802.22274305556 61595.02378472222 63002.22274305556];
assert(max(abs(c.z-z))<1e-8 && isequal(c.x,[43 0 70.5]));
assert(max(abs(c.rz-rz))<1e-8 && isequal(c.rz-c.z,[0 800 0]) && c.yaw==36.5);
pos=perms(1:3); col=perms([1 2 5]); n=0;
for a=1:6
 for b=1:6
  for d=1:6
   rr=pos(a,:); cc=col(b,:); ss=col(d,:);
   done=cycle(1,c,rr,cc,ss,0,0);
   assert(done.placed==3 && done.recovered==3 && size(done.trace,1)==33);
   assert(isequal(done.trace(1:3,1)',[1 1 1]) && isequal(done.trace(1:3,2)',rr));
   assert(isequal(done.trace(1:3,3)',c.x(rr)) && max(abs(done.trace(1:3,4)'-z(rr)))<1e-8);
   for k=1:3
    ring=rr(k); t=done.trace(4+10*(k-1):3+10*k,:);
    assert(isequal(t(:,1)',[17:24 26 27]) && all(t(:,2)==ring));
    assert(max(abs(t(:,3)'-[c.ringYaw(ring) done.trace(k,3) rz(ring) 140 0 0 36.5 find(ss==cc(k)) 160 0]))<1e-8);
   end
   n=n+1;
  end
 end
end
rr=[3 1 2]; cc=[1 5 2]; ss=[2 1 5];
placeOnly=cycle(0,c,rr,cc,ss,0,0);
assert(placeOnly.placed==3 && placeOnly.recovered==0 && size(placeOnly.trace,1)==3);
future=c; future.x(3)=99; future.rz(3)=65000;
resumed=request32(placeOnly,42,true,future,rr,cc,ss);
assert(resumed.placed==3 && resumed.recovered==3 && resumed.trace(5,3)==70.5 && resumed.trace(6,3)==65000);
assert(isequal(placeOnly,request32(placeOnly,43,true,future,rr,cc,ss)));
assert(isequal(placeOnly,request32(placeOnly,42,false,future,rr,cc,ss)));
bad=future; bad.rz(2)=-1; assert(isequal(placeOnly,request32(placeOnly,42,true,bad,rr,cc,ss)));
for b=1:6
 second=col(b,:); [~,ix]=ismember(second,cc); mapped=rr(ix);
 stack=cycle(2,c,mapped,second,ss,0,0);
 assert(stack.recovered==0 && isequal(stack.trace(:,2)',mapped));
 assert(max(abs(stack.trace(:,4)'-(z(mapped)-24000)))<1e-8);
end
sample=cycle(1,c,rr,cc,ss,0,0);
for phase=[1 17:24 26 27]
 for cause=[1 2]  % reference missing or STOP, concurrent with owner completion
  halted=cycle(1,c,rr,cc,ss,phase,cause);
  assert(halted.fault==cause && halted.stop && halted.trace(end,1)==phase && halted.recovered==0);
  assert(isequal(halted.trace,sample.trace(1:size(halted.trace,1),:)));
 end
end
assert(~complete(20,false,500) && ~complete(20,true,199) && complete(20,true,200));
assert(~complete(26,false,900) && ~complete(26,true,599) && complete(26,true,600));
ctl=struct('target',int64([456 -234 1337 -789]),'phase',13,'fault',0,'commands',4);
d=resetDiag(7);
for wheel=1:4
 [after,d]=record(ctl,d,7,wheel,uint8([wheel 253 2 107]),300,[0 299 301 325],true);
 assert(isequal(after,ctl));
end
assert(isequal(d.tick,[0 0 301 325]) && isequal(d.raw{4},uint8([4 253 2 107])));
[after,failed]=record(ctl,d,7,1,uint8([]),300,[0 0 0 0],false);
assert(isequal(after,ctl) && isequal(failed,d));
ctl.phase=99; ctl.fault=12; d=resetDiag(8);
[after,d]=record(ctl,d,7,3,uint8([3 253 2 107]),300,[0 299 301 325],true);
assert(isequal(after,ctl) && d.version==8 && ~any(d.tick) && all(cellfun(@isempty,d.raw)));
report=struct('passed',true,'scope','offline discrete-event model; no firmware execution or hardware', ...
 'matlab',version,'baseline',root,'place_x_mm',c.x,'place_z_pulses',c.z,'recover_x_mm',c.x, ...
 'recover_z_pulses',c.rz,'recover_z_basis','adjusted placement depth plus [0 800 0] virtual pulses by ring; no physical grasp measurement', ...
 'carrier_yaw_deg',c.yaw,'combined_permutations',n,'fault_injections',22, ...
 'grip_deg',140,'grip_wait_ms',200,'release_deg',160,'model_release_wait_ms',600, ...
 'timing_basis','baseline integration10_complete.inc action 5=200 ms and 17=600 ms; modeled owner completion, not measured jaw motion', ...
 'shared_extension_x',true,'request32_locked_xy_yaw_mapping',true,'request32_valid_live_recovery_z',true,'mode0_place_only',true, ...
 'mode2_preserves_60mm_offset',true,'wheel_diag_is_passive',true,'physical_acceptance',false, ...
 'trace_columns',{{'phase','physical_ring','target','place_z'}},'sample_case',sample.trace);
file=fullfile(fileparts(mfilename('fullpath')),'stage3_adjust_recover_matlab.json');
fid=fopen(file,'w','n','UTF-8'); assert(fid>=0); fprintf(fid,'%s\n',jsonencode(report)); fclose(fid);
fprintf('PASS Stage3 MATLAB: %d combined permutations, 22 reference-loss/STOP injections\n',n);
fprintf('Shared place/recover X [%g %g %g]. Physical acceptance pending.\n',c.x);
fprintf('Recovery Z [%.12f %.12f %.12f], placement delta [0 800 0] pulses.\n',c.rz);

function c=adjust(p)
 c.yaw=p.carrier_pick.yaw_deg;
 for k=1:3
  name=sprintf('x%d',k); q=p.rings.(name);
  c.x(k)=q.extension_mm+2*(k==3);
  c.z(k)=q.release_depth_pulses+800*(k==1);
  c.rz(k)=c.z(k)+800*(k==2); c.ringYaw(k)=q.yaw_deg;
 end
end

function s=request32(s,run,ready,live,rr,cc,ss)
 if s.mode==0 && s.placed==3 && s.run==run && ready && ~s.stop && all(live.rz>=3200 & live.rz<=82000)
  c=s.cfg; c.rz=live.rz;
  s=cycle(1,c,rr,cc,ss,0,0,s);
 end
end

function s=cycle(mode,c,rr,cc,ss,failPhase,cause,s)
 if nargin<8
  s=struct('mode',mode,'cfg',c,'run',42,'trace',zeros(0,4),'placed',0,'recovered',0,'fault',0,'stop',false);
 end
 if s.placed==0
  for ring=rr
   s=emit(s,1,ring,c.x(ring),c.z(ring)-24000*(mode==2),failPhase,cause);
   if s.stop, return; end
   s.placed=s.placed+1;
  end
 end
 if mode~=1, return; end
 phase=[17:24 26 27];
 for k=1:3
  assert(s.placed==3);
  ring=rr(k); top=true;
  target=[c.ringYaw(ring) c.x(ring) c.rz(ring) 140 0 0 c.yaw find(ss==cc(k)) 160 0];
  for j=1:numel(phase)
   p=phase(j);
   if p==22 || p==26, assert(top); end
   s=emit(s,p,ring,target(j),0,failPhase,cause);
   if s.stop, return; end
   if p==19, top=false; elseif p==21, top=true; end
  end
  s.recovered=s.recovered+1;
 end
end

function s=emit(s,phase,ring,target,z,failPhase,cause)
 s.trace(end+1,:)=[phase ring target z];
 % Await the owner's completion. The safety decision takes priority over DONE.
 if failPhase==phase, s.fault=cause; s.stop=true; return; end
 elapsed=0;
 while ~complete(phase,true,elapsed), elapsed=elapsed+1; end
end

function yes=complete(phase,done,elapsed)
 yes=done && (phase~=20 || elapsed>=200) && (phase~=26 || elapsed>=600);
end

function d=resetDiag(version)
 d=struct('version',version,'raw',{cell(1,4)},'tick',zeros(1,4));
end

function [ctl,d]=record(ctl,d,version,wheel,raw,ackTick,cacheTicks,writeOK)
 if version~=d.version || ~writeOK, return; end
 d.raw{wheel}=raw;
 if cacheTicks(wheel)>ackTick, d.tick(wheel)=cacheTicks(wheel); end
end
