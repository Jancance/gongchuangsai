function analyze_four_short
% Offline replay of existing wheel/HWT/OD evidence; no tuning or hardware.
out=fileparts(mfilename('fullpath')); diary(fullfile(out,'matlab_diary.log'));
C=jsondecode(fileread(fullfile(out,'contexts.json')));P=jsondecode(fileread(fullfile(out,'parameters.json')));
S=struct('status','RUNNING','matlab_version',version,'hardware_access',false,'parameter_fitted',false,'synchronized_ground_truth',false,'ruler_uncertainty_mm',[],'runs',{{}});
try
for k=1:numel(C)
 c=C(k);d=jsondecode(fileread(c.decoded));h=d.header;tr=double(h.trigger_ms);
 assert(h.quality==0 && h.frozen && ~h.truncated && h.terminal_tail_completed);
 assert(h.wheel_dropped==0 && h.hwt_dropped==0 && h.od_dropped==0);
 assert(numel(d.raw)==h.raw_count && all(diff([d.raw.sample_sequence])==1));
 lo=0;hi=(double(d.raw(end).service_ms)-tr)/1000;ts=cell(1,4);xs=ts;obs=ts;
 for j=1:4
  a=d.wheel(strcmp({d.wheel.source},'position') & [d.wheel.valid] & [d.wheel.wheel]==j-1);
  t=(double([a.source_ms]')-tr)/1000;
  x=double([a.value]').*(1-2*double([a.sign]'))*P.motor_forward_sign(j)*pi*P.wheel_diameter_mm/P.counts_per_rev;
  [t,ix]=unique(t,'last');x=x(ix);assert(numel(t)>20 && all(diff(t)>0) && max(diff(t))<.4);
  ts{j}=t;xs{j}=x;lo=max(lo,t(1));hi=min(hi,t(end));
 end
 hw=d.hwt([d.hwt.healthy] & bitand([d.hwt.valid_mask],1)~=0);
 ht=(double([hw.angle_ms]')-tr)/1000;hy=unwrap(double([hw.yaw_deg]')*pi/180);[ht,ix]=unique(ht,'last');hy=hy(ix);
 lo=max(lo,ht(1));hi=min(hi,ht(end));assert(lo<=0 && hi>c.STOP_s);
 last=floor((hi+1e-10)/.005)*.005;q=(0:.005:last)';q=q(abs(q-c.STOP_s)>1e-9);q=sort([q;c.STOP_s]);
 pos=zeros(numel(q),4);for j=1:4,pos(:,j)=interp1(ts{j},xs{j},q,'linear');end
 yaw=interp1(ht,hy,q,'linear');ref=(h.path_heading_deg-h.heading_offset_deg)*pi/180;
 path=zeros(numel(q),2);body=path;parts=path;
 for n=2:numel(q)
  dw=pos(n,:)-pos(n-1,:);F=mean(dw)*P.forward_scale;L=(dw(1)-dw(2)-dw(3)+dw(4))/4;
  half=(yaw(n)-yaw(n-1))/2;ang=(yaw(n)+yaw(n-1))/2-ref;si=1;if abs(half)>1e-10,si=sin(half)/half;end
  path(n,:)=path(n-1,:)+si*[F*cos(ang)-L*sin(ang),F*sin(ang)+L*cos(ang)];
  body(n,:)=body(n-1,:)+[F L];parts(n,:)=parts(n-1,:)+si*[-L*sin(ang) F*sin(ang)];
 end
 assert(all(isfinite(path(:))));
 switch c.direction
  case 'forward',axis=1;sgn=1;cross=2;
  case 'backward',axis=1;sgn=-1;cross=2;
  case 'left',axis=2;sgn=1;cross=1;
  case 'right',axis=2;sgn=-1;cross=1;
 end
 m=sgn*path(:,axis);x=path(:,cross);i=find(abs(q-c.STOP_s)<1e-10);assert(numel(i)==1);
 ep=c.endpoint;pe=ep.translation_travel_mm;if axis==1,pe=pe*P.forward_scale;end
 r=struct('name',c.name,'direction',c.direction,'target_mm',c.target_mm,'primary',c.primary,'axf_sha256',c.axf_sha256,...
  'ground_main_mm',c.ground_main_mm,'ground_cross_mm',c.ground_cross_mm,'ground_error_mm',c.ground_main_mm-c.target_mm,...
  'frozen_progress_mm',c.frozen_progress_mm,'STOP_s',c.STOP_s,'model_STOP_main_mm',m(i),'common_end_s',q(end),...
  'model_end_main_mm',m(end),'model_end_cross_mm',x(end),'model_minus_ground_main_mm',m(end)-c.ground_main_mm,...
  'model_minus_ground_cross_mm',x(end)-c.ground_cross_mm,'model_STOP_to_end_main_mm',m(end)-m(i),...
  'model_STOP_to_end_cross_mm',x(end)-x(i),'terminal_s',c.terminal_s,'terminal_minus_common_end_s',c.terminal_s-q(end),...
  'first_persistent_reported_zero_s',c.persistent_zero_s,'body_main_mm',sgn*body(end,axis),...
  'yaw_effect_on_main_mm',sgn*(path(end,axis)-body(end,axis)),'relative_yaw_start_end_peak_deg',(180/pi)*[yaw(1)-ref yaw(end)-ref max(abs(yaw-ref))],...
  'PE3_projected_scaled_once_mm',pe,'PE3_end_span_ms',max([ep.end_ms1 ep.end_ms2 ep.end_ms3 ep.end_ms4])-min([ep.end_ms1 ep.end_ms2 ep.end_ms3 ep.end_ms4]),...
  'raw_count',h.raw_count,'source_common_limit_s',hi,'physical_coast_established',false);
 od=d.od;ok=[od.valid] & [od.reasons]==0 & [od.continuity_lost]==0;v=od(ok);
 ot=(double([v.sample_ms]')-tr)/1000;[ot,ix]=unique(ot,'last');v=v(ix);theta=h.path_heading_deg*pi/180;
 dx=double([v.x_mm]')-h.origin_x_mm;dy=double([v.y_mm]')-h.origin_y_mm;
 fl=[dx*cos(theta)+dy*sin(theta),-dx*sin(theta)+dy*cos(theta)];
 match=find(q>=ot(1) & q<=ot(end));assert(~isempty(match));z=match(end);oval=interp1(ot,fl,q(z),'linear');
 r.OD=struct('valid_count',sum(ok),'total_count',numel(od),'valid_source_end_s',ot(end),'matched_s',q(z),...
 'matched_main_mm',sgn*oval(axis),'wheel_same_time_main_mm',m(z),'matched_cross_mm',oval(cross),'wheel_same_time_cross_mm',x(z),'last_invalid_excluded',~od(end).valid);
 latest=zeros(1,4);aa=cell(1,4);
 for j=1:4
  a=d.wheel(strcmp({d.wheel.source},'position') & [d.wheel.valid] & [d.wheel.wheel]==j-1 & [d.wheel.observed_ms]<=h.stop_ms);
  aa{j}=a;latest(j)=a(end).source_ms;
 end
 ct=min(latest);aligned=zeros(1,4);
 for j=1:4
  a=aa{j};t=double([a.source_ms]');v=double([a.value]').*(1-2*double([a.sign]'));[t,ix]=unique(t,'last');v=v(ix);
  base=ep.(sprintf('start_raw%d',j))*(1-2*ep.(sprintf('start_sign%d',j)));
  aligned(j)=(interp1(t,v,ct,'linear')-base)*P.motor_forward_sign(j)*pi*P.wheel_diameter_mm/P.counts_per_rev;
 end
 alignedFL=[mean(aligned)*P.forward_scale,(aligned(1)-aligned(2)-aligned(3)+aligned(4))/4];
 r.reconstructed_frozen_progress_mm=sgn*alignedFL(axis);
 r.progress_reconstruction_error_mm=r.reconstructed_frozen_progress_mm-c.frozen_progress_mm;
 r.progress_source_time_s=(ct-tr)/1000;r.progress_age_at_STOP_ms=double(h.stop_ms)-ct;
 assert(abs(r.progress_reconstruction_error_mm)<.01);
 r.extended=extended_end(c,d,P,axis,sgn,cross);
 r.first_reached_telemetry_s=c.first_reached_s;
 S.runs{k}=r;
 T=table(q,m,x,sgn*body(:,axis),(yaw-ref)*180/pi,'VariableNames',{'source_s','model_main_mm','model_cross_mm','body_main_mm','yaw_deg'});
 writetable(T,fullfile(out,[c.name '_series.csv']));
end
T=struct2table([S.runs{:}]);T.OD=[];T.extended=[];T.relative_yaw_start_end_peak_deg=[];writetable(T,fullfile(out,'summary.csv'));
f=figure('Visible','off','Color','w','Position',[10 10 1200 700]);
targets=[1500 300];for j=1:2
 subplot(1,2,j);hold on;grid on;
 for k=1:numel(C)
  r=S.runs{k};if ~r.primary || r.target_mm~=targets(j),continue;end
  plot(r.ground_error_mm,r.model_minus_ground_main_mm,'o','DisplayName',r.name);
 end
 xlabel('Ruler minus target (mm)');ylabel('Model end minus final ruler (mm)');title(sprintf('Target %d mm; source-window limits apply',targets(j)));legend('Location','eastoutside','Interpreter','none','FontSize',6);
end
print(f,fullfile(out,'main_distance_errors.png'),'-dpng','-r130');close(f);
S.status='PASS_WITH_LIMITATIONS';
catch e
 S.status='FAIL';S.error=getReport(e,'extended','hyperlinks','off');writejson(out,S);diary off;rethrow(e);
end
writejson(out,S);disp('FOUR_DIRECTION_SHORT_ANALYSIS_PASS_WITH_LIMITATIONS');diary off;
end
function writejson(out,S)
fid=fopen(fullfile(out,'summary.json'),'w','n','UTF-8');fwrite(fid,jsonencode(S),'char');fclose(fid);
end

function z=extended_end(c,d,P,axis,sgn,cross)
tr=double(d.header.trigger_ms);ts=cell(1,4);xs=ts;hi=Inf;ep=c.endpoint;
for j=1:4
 a=d.wheel(strcmp({d.wheel.source},'position') & [d.wheel.valid] & [d.wheel.wheel]==j-1);
 t=double([a.source_ms]');x=double([a.value]').*(1-2*double([a.sign]'));
 for k=1:numel(c.pm_positions)
  pm=c.pm_positions(k);
  if bitand(pm.valid_mask,2^(j-1))~=0
   t(end+1)=pm.(sprintf('sample_ms%d',j));x(end+1)=pm.(sprintf('raw%d',j))*(1-2*pm.(sprintf('sign%d',j)));
  end
 end
 t(end+1)=ep.(sprintf('end_ms%d',j));x(end+1)=ep.(sprintf('end_raw%d',j))*(1-2*ep.(sprintf('end_sign%d',j)));
 [t,ix]=unique(t,'last');x=x(ix);ts{j}=(t-tr)/1000;xs{j}=x*P.motor_forward_sign(j)*pi*P.wheel_diameter_mm/P.counts_per_rev;hi=min(hi,ts{j}(end));
end
hw=d.hwt([d.hwt.healthy] & bitand([d.hwt.valid_mask],1)~=0);
ht=(double([hw.angle_ms]')-tr)/1000;hy=unwrap(double([hw.yaw_deg]')*pi/180);[ht,ix]=unique(ht,'last');hy=hy(ix);hi=min(hi,ht(end));
q=(0:.005:floor((hi+1e-10)/.005)*.005)';pos=zeros(numel(q),4);
for j=1:4,assert(q(1)>=ts{j}(1) && q(end)<=ts{j}(end));pos(:,j)=interp1(ts{j},xs{j},q,'linear');end
yaw=interp1(ht,hy,q,'linear');ref=(d.header.path_heading_deg-d.header.heading_offset_deg)*pi/180;
dw=diff(pos);F=mean(dw,2)*P.forward_scale;L=(dw(:,1)-dw(:,2)-dw(:,3)+dw(:,4))/4;
half=diff(yaw)/2;ang=(yaw(2:end)+yaw(1:end-1))/2-ref;si=ones(size(half));nz=abs(half)>1e-10;si(nz)=sin(half(nz))./half(nz);
fl=[sum(si.*(F.*cos(ang)-L.*sin(ang))),sum(si.*(F.*sin(ang)+L.*cos(ang)))];
z=struct('common_end_s',q(end),'main_mm',sgn*fl(axis),'cross_mm',fl(cross),'model_minus_ground_main_mm',sgn*fl(axis)-c.ground_main_mm,'terminal_minus_end_s',c.terminal_s-q(end),'method','RAW + valid per-wheel PM1 + PE3 source positions; HWT bracketed only; no extrapolation; PE3 asynchronous span remains explicit');
end
