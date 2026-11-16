function validate_drift_candidate
% Frozen beta candidate mathematics and existing200Hz logs; no hardware/truth.
out=fileparts(mfilename('fullpath'));diary(fullfile(out,'matlab_diary.log'));
beta=.04;f=pi*77/60;follow=2;
A=150/(1+beta)-f*4;J=300/(1+beta)-f*12;v=400;distance=2100;
tj=A/J;ta=v/A-tj;tv=(distance-v*(2*tj+ta))/v;
assert(A>0 && J>0 && ta>0 && tv>0);
d=[tj ta tj tv tj ta tj];b=[0 cumsum(d)];js=J*[1 0 -1 0 -1 0 1];x=zeros(8,3);
for k=1:7
 dt=d(k);p=x(k,1);s=x(k,2);a=x(k,3);j=js(k);
 x(k+1,:)=[p+s*dt+.5*a*dt^2+j*dt^3/6,s+a*dt+.5*j*dt^2,a+j*dt];
end
assert(abs(x(end,1)-2100)<1e-8 && max(abs(x(end,2:3)))<1e-9);
assert(b(end)+2<12.779 && abs(x(4,2)-400)<1e-9);
assert((A+f*4)*(1+beta)<=150+1e-9 && (J+f*12)*(1+beta)<=300+1e-9);
t=unique([(0:.002:b(end))';b']);trace=zeros(numel(t),8);
for n=1:numel(t)
 k=find(t(n)<b(2:end),1);
 if isempty(k),p=2100;s=0;a=0;j=0;else
  dt=t(n)-b(k);j=js(k);p=x(k,1)+x(k,2)*dt+.5*x(k,3)*dt^2+j*dt^3/6;
  s=x(k,2)+x(k,3)*dt+.5*j*dt^2;a=x(k,3)+j*dt;
 end
 base=round(s/f);trim=round(beta*base);trace(n,:)=[t(n) p s a j base trim -trim*f];
end
assert(all(diff(trace(:,2))>=-1e-9) && max(trace(:,3))>=400-1e-9);
maxwheel=0;maxtrim=0;commandCases=0;
for base=0:102
 trim=round(beta*base);maxtrim=max(maxtrim,trim);
 for req=[-2 0 2]
  h=max(-min(base,2),min(min(base,2),req));
  w=[base-trim-h,-base-trim+h,-base-trim-h,base-trim+h];
  assert(abs(sum(w)/4+trim)<1e-10);
  assert(abs((w(1)-w(2)-w(3)+w(4))/4-base)<1e-10);
  assert(all(abs(w)<=108) && all(abs(w)<130));
  if base>2,assert(all(w([1 4])>=0) && all(w([2 3])<=0));end
  if base==0,assert(all(w==0));end
  maxwheel=max(maxwheel,max(abs(w)));commandCases=commandCases+1;
 end
end
assert(maxwheel==108 && maxtrim==4);
adjacent=[];
for heading=[-2 0 2]
 prevOld=zeros(1,4);prevNew=zeros(1,4);
 for base=0:102
  h=max(-min(base,2),min(min(base,2),heading));trim=round(beta*base);
  old=[base-h,-base+h,-base-h,base+h];new=old-trim;
  if base>0,adjacent=[adjacent;base heading max(abs(old-prevOld)) max(abs(new-prevNew)) trim-round(beta*(base-1))];end %#ok<AGROW>
  prevOld=old;prevNew=new;
 end
end
assert(max(abs(adjacent(:,end)))==1 && max(adjacent(:,4))<=3);
profileWheels=[trace(:,1),trace(:,6)*[1 -1 -1 1],trace(:,6)*[1 -1 -1 1]-trace(:,7)];
jump=find(diff(trace(:,7))~=0)+1;
trimJumps=[trace(jump,1),trace(jump-1,6:7),trace(jump,6:7),trace(jump,7)-trace(jump-1,7)];
assert(all(abs(trimJumps(:,end))==1));
% Common-time orthogonal wheel increments: common backward term cancels left.
for dl=[-2 0 17],for df=[-4 0 9],for dh=[-3 0 2]
 w=[dl+df-dh,-dl+df+dh,-dl+df-dh,dl+df+dh];
 assert(abs((w(1)-w(2)-w(3)+w(4))/4-dl)<1e-10);
end,end,end
dataSummary=struct([]);
for run=1:2
 file=fullfile(out,'..',sprintf('lateral-field-run%d',run),'capture','decoded.json');q=jsondecode(fileread(file));
 assert(numel(q.raw)==2032 && q.header.quality==0 && q.header.raw_count==2032);
 tr=(double([q.raw.service_ms]')-q.header.trigger_ms)/1000;
 assert(all(diff(tr)>0) && max(diff(tr))<=.02);
 theta=q.header.path_heading_deg*pi/180;
 od=q.od;dx=double([od.x_mm]')-q.header.origin_x_mm;dy=double([od.y_mm]')-q.header.origin_y_mm;
 F=dx*cos(theta)+dy*sin(theta);L=-dx*sin(theta)+dy*cos(theta);
 ot=(double([od.observed_ms]')-q.header.trigger_ms)/1000;
 ids=arrayfun(@(tt)find(abs(ot-tt)==min(abs(ot-tt)),1),[3 5 7 9.5]);
 % RAW body front is -sensorY. Mean acceleration is not displacement truth.
 ar=-double([q.raw.ay]')*.001197100830078125;
 means=zeros(4,3);edges=[0 3 5 7 9.5];
 for k=1:4,mask=tr>=edges(k)&tr<edges(k+1);means(k,:)=[edges(k) edges(k+1) mean(ar(mask))];end
 dataSummary(run).run=run;dataSummary(run).raw_count=numel(q.raw);dataSummary(run).dt_min_max_s=[min(diff(tr)) max(diff(tr))];
 dataSummary(run).od_stages=[ot(ids) F(ids) L(ids)];dataSummary(run).body_front_raw_accel_stage_mean_m_s2=means;
 dataSummary(run).raw_not_ground_truth=true;
 c=readtable(fullfile(out,'..','lateral-distance-diagnosis','telemetry_state_changes.csv'));
 c=c(c.run==run,:);activeBase=c.base_rpm(c.backend_state<7);
 dataSummary(run).actual_stage_base_trim=[activeBase round(beta*activeBase)];
 dataSummary(run).stop_residual_trim=round(beta*0);
end
summary=struct('status','PASS_WITH_LIMITATIONS','matlab_version',version,'beta',beta,...
 'A_reference',A,'J_reference',J,'profile_duration_s',b(end),'phase_duration_s',d,...
 'reference_peak_mm_s',max(trace(:,3)),'wheel_peak_rpm',maxwheel,'trim_peak_rpm',maxtrim,...
 'command_cases',commandCases,'data',dataSummary,'hardware_used',false,...
 'trim_jump_count',numel(jump),'trim_jump_magnitude_rpm',unique(abs(trimJumps(:,end)))',...
 'continuous_budget_only',true,'integer_steps_have_finite_continuous_jerk',false,...
 'limits',{{'4percent is experimental feedforward, not proven ground correction',...
 '400 is left reference, integer RPM and slip can change actual speed',...
 'No ground truth from RAW acceleration integration',...
 'Integer trim steps remain discontinuous, not continuous jerk certified',...
 'Left distance and STOP thresholds unchanged; no hardware test performed'}});
writematrix(trace,fullfile(out,'candidate_profile_trace.csv'));
writematrix(trimJumps,fullfile(out,'integer_trim_jumps.csv'));
writematrix(adjacent,fullfile(out,'adjacent_command_changes.csv'));
writematrix(profileWheels,fullfile(out,'integer_profile_wheels.csv'));
oldA=150-f*4;oldJ=300-f*12;oldTj=oldA/oldJ;oldTa=v/oldA-oldTj;
summary.original_profile_duration_s=2*(2*oldTj+oldTa)+(distance-v*(2*oldTj+oldTa))/v;
fid=fopen(fullfile(out,'matlab_summary.json'),'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(summary));fclose(fid);
disp(summary);diary off;
end
