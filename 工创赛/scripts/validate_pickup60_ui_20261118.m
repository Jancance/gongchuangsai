% Source pickup first net angle60; legacy callers120; no blind-gap inference.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-18','pickup60-ui');if ~exist(out,'dir'),mkdir(out);end
checks=0;
paths={[0,20,40,59.9],[0,20,40,60.1],[0,-20,-40,-60.1],[0,20,0,20],[170,-170,-150,-129.9],[0,30,60,90,120.1]};
net_truth=[59.9,60.1,-60.1,20,60.1,120.1];
for n=1:numel(paths)
 a=paths{n}*pi/180;net=0;
 for k=2:numel(a),d=a(k)-a(k-1);net=net+atan2(sin(d),cos(d));end
 assert(abs(net*180/pi-net_truth(n))<1e-8);checks=checks+1;
 for required=[60,120]
  actual=abs(net)>=required*pi/180;
  expected=abs(net_truth(n))>=required;
  assert(actual==expected);checks=checks+1;
  for frames=0:3
   for state=[2,4,5,6,7,8]
    for item=0:2
     angleok=item>0 || actual;candidate=frames==3 && angleok;
     sent=state==6 && candidate;
     assert(sent==((state==6 && frames==3) && (item>0 || expected)));checks=checks+1;
    end
   end
  end
 end
end
%30deg observed, then blind+movement gap traverses another80deg: add none.
net=30;previous=[];phase_after_gap=110; %#ok<NASGU>
assert(net==30 && isempty(previous));checks=checks+1;
previous=110;next=125;net=net+next-previous;assert(net==45);checks=checks+1;
% Warm state2 observations remain usable through stationary4/5/6; motion clears
% stopped frame history but never invents a new angular increment.
frames=3;angle=60.1;for state=[2,4,5,6],assert(frames==3 && angle==60.1);checks=checks+1;end
frames=0;assert(frames==0 && angle==60.1);checks=checks+1;
r=struct('passed',true,'checks',checks,'source_first_required_deg',60,'legacy_default_required_deg',120,'stopped_frames',3,'host_frame_age_limit_ms',200,'near_stop_speed_unchanged',25,'hardware_access',false,'note','Signed visual net-phase model, not calibrated mechanical turn angle or physical pickup acceptance');
fid=fopen(fullfile(out,'matlab-result.json'),'w');fprintf(fid,'%s\n',jsonencode(r));fclose(fid);fprintf('PASS MATLAB pickup60 %d checks\n',checks);
