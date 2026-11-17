function analyze_three_directions
out=fileparts(mfilename('fullpath'));B=fileparts(out);C=jsondecode(fileread(fullfile(out,'contexts.json')));P=jsondecode(fileread(fullfile(B,'four-direction-short-analysis','parameters.json')));S=struct('status','RUNNING','matlab_version',version,'hardware_access',false,'product_changed',false,'counterfactual_control_verified',false,'ground_synchronized',false);rows=cell(numel(C),1);
for k=1:numel(C)
 c=C(k);r=struct('name',c.name,'direction',c.direction,'raw_available',c.raw_available,'target_mm',c.target_mm,'ground_main_mm',c.ground_main_mm,'ground_cross_mm',c.ground_cross_mm,'main_mm',NaN,'cross_mm',NaN,'body_F_mm',NaN,'body_L_mm',NaN,'yaw_effect_F_mm',NaN,'yaw_effect_L_mm',NaN,'common_end_s',NaN,'terminal_gap_s',NaN,'yaw_start_deg',NaN,'yaw_end_deg',NaN,'yaw_peak_deg',NaN,'raw_count',0);
 if c.raw_available
  d=jsondecode(fileread(c.decoded));h=d.header;tr=double(h.trigger_ms);assert(h.quality==0 && h.frozen && ~h.truncated && h.terminal_tail_completed);assert(all([h.wheel_dropped h.hwt_dropped h.od_dropped]==0));ts=cell(1,4);xs=ts;lo=0;hi=(double(d.raw(end).service_ms)-tr)/1000;
  for j=1:4
   a=d.wheel(strcmp({d.wheel.source},'position') & [d.wheel.valid] & [d.wheel.wheel]==j-1);t=(double([a.source_ms]')-tr)/1000;x=double([a.value]').*(1-2*double([a.sign]'))*P.motor_forward_sign(j)*pi*P.wheel_diameter_mm/P.counts_per_rev;[t,ix]=unique(t,'last');ts{j}=t;xs{j}=x(ix);lo=max(lo,t(1));hi=min(hi,t(end));
  end
  a=d.hwt([d.hwt.healthy] & bitand([d.hwt.valid_mask],1)~=0);ht=(double([a.angle_ms]')-tr)/1000;hy=unwrap(double([a.yaw_deg]')*pi/180);[ht,ix]=unique(ht,'last');hy=hy(ix);lo=max(lo,ht(1));hi=min(hi,ht(end));assert(lo<=0);q=(0:.005:floor((hi+1e-10)/.005)*.005)';assert(q(end)-hi<1e-9);q(end)=min(q(end),hi);pos=zeros(numel(q),4);for j=1:4,pos(:,j)=interp1(ts{j},xs{j},q);end
  yaw=interp1(ht,hy,q);ref=(h.path_heading_deg-h.heading_offset_deg)*pi/180;dw=diff(pos);F=mean(dw,2)*P.forward_scale;L=(dw(:,1)-dw(:,2)-dw(:,3)+dw(:,4))/4;half=diff(yaw)/2;mid=(yaw(2:end)+yaw(1:end-1))/2-ref;si=ones(size(half));nz=abs(half)>1e-10;si(nz)=sin(half(nz))./half(nz);path=[sum(si.*(F.*cos(mid)-L.*sin(mid))),sum(si.*(F.*sin(mid)+L.*cos(mid)))];body=[sum(F),sum(L)];assert(all(isfinite([path body])));
  if strcmp(c.direction,'right'),r.main_mm=-path(2);r.cross_mm=path(1);elseif strcmp(c.direction,'backward'),r.main_mm=-path(1);r.cross_mm=path(2);else,r.main_mm=path(1);r.cross_mm=path(2);end
  r.body_F_mm=body(1);r.body_L_mm=body(2);r.yaw_effect_F_mm=path(1)-body(1);r.yaw_effect_L_mm=path(2)-body(2);r.common_end_s=q(end);r.terminal_gap_s=(double(h.first_terminal_receiver_ms)-tr)/1000-q(end);r.yaw_start_deg=(yaw(1)-ref)*180/pi;r.yaw_end_deg=(yaw(end)-ref)*180/pi;r.yaw_peak_deg=max(abs(yaw-ref))*180/pi;r.raw_count=h.raw_count;
 end
 rows{k}=r;
end
R=[rows{:}];
for k=1:numel(R)
 R(k).main_residual_mm=R(k).ground_main_mm-R(k).main_mm;
 R(k).cross_residual_mm=R(k).ground_cross_mm-R(k).cross_mm;
end
S.runs=R; S.selection='forward trial2 excluded by explicit user instruction; original retained';
S.cross_sign='right trials: forward positive; forward/backward trials: left positive';
S.status='PASS_OFFLINE_DIAGNOSTIC_ONLY';
writetable(struct2table(R),fullfile(out,'replay_summary.csv'));
fid=fopen(fullfile(out,'summary.json'),'w','n','UTF-8');fwrite(fid,jsonencode(S),'char');fclose(fid);
disp(S.status);disp(struct2table(R));
end
