function audit_localization
out=fileparts(mfilename('fullpath'));base=fileparts(fileparts(out));old=fullfile(base,'four-direction-short-analysis');diary(fullfile(out,'matlab_diary.log'));
S=struct('status','RUNNING','matlab_version',version,'hardware_access',false,'diagnostic_not_deployable',true,'ground_truth_synchronized',false,'initial_placement_error_known',false,'terminal_window_aligned_to_ruler',false);
try
S.analytic=analytic();C=jsondecode(fileread(fullfile(old,'contexts.json')));P=jsondecode(fileread(fullfile(old,'parameters.json')));rows=struct([]);
for k=1:numel(C)
c=C(k);d=jsondecode(fileread(c.decoded));h=d.header;tr=double(h.trigger_ms);
assert(h.quality==0 && h.frozen && ~h.truncated && h.terminal_tail_completed);assert(h.wheel_dropped==0 && h.hwt_dropped==0 && h.od_dropped==0);assert(all(diff([d.raw.sample_sequence])==1));
ts=cell(1,4);xs=ts;lo=0;hi=(double(d.raw(end).service_ms)-tr)/1000;
for j=1:4
 a=d.wheel(strcmp({d.wheel.source},'position') & [d.wheel.valid] & [d.wheel.wheel]==j-1);t=(double([a.source_ms]')-tr)/1000;x=double([a.value]').*(1-2*double([a.sign]'))*P.motor_forward_sign(j)*pi*P.wheel_diameter_mm/P.counts_per_rev;
 [t,ix]=unique(t,'last');ts{j}=t;xs{j}=x(ix);lo=max(lo,t(1));hi=min(hi,t(end));
end
a=d.hwt([d.hwt.healthy] & bitand([d.hwt.valid_mask],1)~=0);ht=(double([a.angle_ms]')-tr)/1000;hy=unwrap(double([a.yaw_deg]')*pi/180);[ht,ix]=unique(ht,'last');hy=hy(ix);lo=max(lo,ht(1));hi=min(hi,ht(end));assert(lo<=0 && hi>c.STOP_s);
q=(0:.005:floor((hi+1e-10)/.005)*.005)';q=sort([q(abs(q-c.STOP_s)>1e-9);c.STOP_s]);pos=zeros(numel(q),4);for j=1:4,pos(:,j)=interp1(ts{j},xs{j},q);end
yaw=interp1(ht,hy,q);ref=(h.path_heading_deg-h.heading_offset_deg)*pi/180;dw=diff(pos);F=mean(dw,2)*P.forward_scale;L=(dw(:,1)-dw(:,2)-dw(:,3)+dw(:,4))/4;
[path,fp,lp]=integ(F,L,yaw-ref);body=[0 0;cumsum([F L])];[axis,sgn,cross]=axes_for(c.direction);wrong=(rot(yaw(end)-ref)*body(end,:)')';
r=struct('name',c.name,'direction',c.direction,'axf_sha256',c.axf_sha256,'target_mm',c.target_mm,'current_version',startsWith(c.axf_sha256,'6b4c42de'),'end_s',q(end),'STOP_s',c.STOP_s,'terminal_gap_s',c.terminal_s-q(end),'ground_main_mm',c.ground_main_mm,'ground_cross_mm',c.ground_cross_mm,'main_mm',sgn*path(end,axis),'cross_mm',path(end,cross),'body_F_mm',body(end,1),'body_L_mm',body(end,2),'F_rotated_F_mm',fp(end,1),'F_rotated_L_mm',fp(end,2),'L_rotated_F_mm',lp(end,1),'L_rotated_L_mm',lp(end,2),'yaw_only_main_mm',sgn*(path(end,axis)-body(end,axis)),'yaw_only_cross_mm',path(end,cross)-body(end,cross),'wrong_final_yaw_main_mm',sgn*wrong(axis),'wrong_final_yaw_cross_mm',wrong(cross));
r.main_residual_mm=r.main_mm-c.ground_main_mm;r.cross_residual_mm=r.cross_mm-c.ground_cross_mm;
od=d.od;od=od([od.valid] & [od.reasons]==0 & [od.continuity_lost]==0);ot=(double([od.sample_ms]')-tr)/1000;[ot,ix]=unique(ot,'last');od=od(ix);theta=h.path_heading_deg*pi/180;dx=double([od.x_mm]')-h.origin_x_mm;dy=double([od.y_mm]')-h.origin_y_mm;fl=[dx*cos(theta)+dy*sin(theta),-dx*sin(theta)+dy*cos(theta)];z=find(q>=ot(1) & q<=ot(end),1,'last');v=interp1(ot,fl,q(z));r.OD_match_s=q(z);r.OD_same_time_F_error_mm=path(z,1)-v(1);r.OD_same_time_L_error_mm=path(z,2)-v(2);assert(max(abs([r.OD_same_time_F_error_mm r.OD_same_time_L_error_mm]))<.01);assert(max(abs(path(:)-fp(:)-lp(:)))<1e-8);rows(k)=r;
T=table(q,path(:,1),path(:,2),body(:,1),body(:,2),fp(:,1),fp(:,2),lp(:,1),lp(:,2),(yaw-ref)*180/pi,'VariableNames',{'source_s','path_F_mm','path_L_mm','body_F_mm','body_L_mm','F_project_F_mm','F_project_L_mm','L_project_F_mm','L_project_L_mm','yaw_deg'});writetable(T,fullfile(out,[c.name '_decomposition.csv']));
end
S.runs=rows;writetable(struct2table(rows),fullfile(out,'run_summary.csv'));S.loo=loo(rows,out);S.turns=turns(base,out);writetable(struct2table(S.loo),fullfile(out,'loo_summary.csv'));writetable(struct2table(S.turns),fullfile(out,'turn_summary.csv'));S.status='PASS_DIAGNOSTIC_WITH_LIMITATIONS';
catch e
S.status='FAIL';S.error=getReport(e,'extended','hyperlinks','off');savejson(out,S);diary off;rethrow(e);
end
savejson(out,S);disp(S.status);diary off;
end
function [p,fp,lp]=integ(F,L,yaw)
half=diff(yaw)/2;mid=(yaw(2:end)+yaw(1:end-1))/2;si=ones(size(half));nz=abs(half)>1e-10;si(nz)=sin(half(nz))./half(nz);fp=[0 0;cumsum([si.*F.*cos(mid),si.*F.*sin(mid)],1)];lp=[0 0;cumsum([-si.*L.*sin(mid),si.*L.*cos(mid)],1)];p=fp+lp;
end
function R=rot(a),R=[cos(a) -sin(a);sin(a) cos(a)];end
function [a,s,c]=axes_for(d)
switch d
case 'forward',a=1;s=1;c=2;
case 'backward',a=1;s=-1;c=2;
case 'left',a=2;s=1;c=1;
case 'right',a=2;s=-1;c=1;
end
end
function A=analytic
cases={100,0,[0;0],[100 0];-100,0,[0;0],[-100 0];0,100,[0;0],[0 100];0,-100,[0;0],[0 -100];100,0,[pi/3;pi/3],[50 50*sqrt(3)];100,0,[0;pi/2],[200/pi 200/pi]};err=zeros(size(cases,1),1);
for k=1:size(cases,1),p=integ(cases{k,1},cases{k,2},cases{k,3});err(k)=max(abs(p(end,:)-cases{k,4}));end
y=unwrap([179;-179]*pi/180);p=integ(100,0,y);err(end+1)=max(abs(p(end,:)-[-100*sin(pi/180)/(pi/180),0]));y=[0;0;89.8;89.8]*pi/180;p=integ([300;0;300],[0;0;0],y);expected=[300+300*cosd(89.8),300*sind(89.8)];err(end+1)=max(abs(p(end,:)-expected));double_count=[300 300];wrong=(rot(y(end))*[600;0])';assert(max(err)<1e-10);assert(norm(double_count-expected)>.999);assert(norm(wrong-expected)>400);A=struct('cases',numel(err),'max_abs_error_mm',max(err),'continuous_actual89_8_end_FL_mm',expected,'wrong_extra0_2_end_FL_mm',double_count,'extra0_2_error_mm',norm(double_count-expected),'wrong_final_yaw_end_FL_mm',wrong,'all_pass',true);
end
function R=loo(rows,out)
names={'forward','backward','left','right'};R=struct([]);A=[];B=[];D=[];details=struct([]);n=0;
for g=1:4
ix=find([rows.current_version] & strcmp({rows.direction},names{g}));assert(numel(ix)>=2);b=[rows(ix).cross_residual_mm]';x=[rows(ix).main_mm]';f=zeros(size(b));s=f;
for j=1:numel(ix)
train=setdiff(1:numel(ix),j);alpha=mean(b(train));beta=sum(x(train).*b(train))/sum(x(train).^2);f(j)=b(j)-alpha;s(j)=b(j)-beta*x(j);n=n+1;details(n)=struct('held_out_name',rows(ix(j)).name,'direction',names{g},'training_n',numel(train),'training_names',strjoin({rows(ix(train)).name},';'),'fixed_train_mm',alpha,'proportional_train_ratio',beta,'baseline_residual_mm',b(j),'fixed_LOO_residual_mm',f(j),'proportional_LOO_residual_mm',s(j));
end
R(g)=metrics(names{g},b,f,s);A=[A;b];B=[B;f];D=[D;s];
end
R(5)=metrics('all_current16',A,B,D);assert(numel(A)==16);writetable(struct2table(details),fullfile(out,'loo_heldout.csv'));
end
function r=metrics(name,b,f,s)
r=struct('group',name,'n',numel(b),'baseline_MAE_mm',mean(abs(b)),'fixed_LOO_MAE_mm',mean(abs(f)),'proportional_LOO_MAE_mm',mean(abs(s)),'baseline_worst_mm',max(abs(b)),'fixed_LOO_worst_mm',max(abs(f)),'proportional_LOO_worst_mm',max(abs(s)),'diagnostic_not_deployable',true);
end
function R=turns(base,out)
names={'turn90_90-field-retry2-keepalive3s','turn90_90-field-repeat-ccw2','turn90_90-field-cw1-keepalive3s','turn90_90-field-repeat-cw2'};R=struct([]);
for k=1:4
d=jsondecode(fileread(fullfile(base,names{k},'capture','decoded.json')));h=d.header;a=d.raw;assert(h.quality==0 && ~h.truncated && all(diff([a.sample_sequence])==1));cyc=double([a.edge_cycle]');dt=mod(diff(cyc),2^32)/double(h.system_core_clock);assert(all(dt>0 & dt<.02));t=(double(a(1).service_ms)-double(h.trigger_ms))/1000+[0;cumsum(dt)];rate=-double([a.gz]')*1000/32768;pre=t>=-.4 & t<=-.1;assert(sum(pre)>=50);bias=mean(rate(pre));gy=cumtrapz(t,rate-bias);hw=d.hwt([d.hwt.healthy] & bitand([d.hwt.valid_mask],1)~=0);ht=(double([hw.angle_ms]')-double(h.trigger_ms))/1000;hy=unwrap(double([hw.yaw_deg]')*pi/180)*180/pi;[ht,ix]=unique(ht,'last');hy=hy(ix);lo=max([0 t(1) ht(1)]);hi=min(t(end),ht(end));q=t(t>=lo & t<=hi);igy=interp1(t,gy,q)-interp1(t,gy,lo);hwy=interp1(ht,hy,q)-interp1(ht,hy,lo);R(k)=struct('name',names{k},'cal_samples',sum(pre),'PRE_gyro_z_mean_dps',bias,'start_s',lo,'end_s',q(end),'ICM_delta_yaw_deg',igy(end),'HWT_delta_yaw_deg',hwy(end),'ICM_minus_HWT_deg',igy(end)-hwy(end),'max_abs_difference_deg',max(abs(igy-hwy)),'max_native_dt_s',max(dt),'end_alignment_forced',false);writetable(table(q,igy,hwy,igy-hwy,'VariableNames',{'source_s','ICM_delta_yaw_deg','HWT_delta_yaw_deg','difference_deg'}),fullfile(out,[names{k} '_yaw.csv']));
end
end
function savejson(out,S)
fid=fopen(fullfile(out,'summary.json'),'w','n','UTF-8');fwrite(fid,jsonencode(S),'char');fclose(fid);
end
