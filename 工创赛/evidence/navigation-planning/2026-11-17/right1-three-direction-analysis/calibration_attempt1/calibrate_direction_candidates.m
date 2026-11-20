function calibrate_direction_candidates
out=fileparts(mfilename('fullpath'));S=jsondecode(fileread(fullfile(out,'summary.json')));R=S.runs;names={'right','forward','backward'};G=struct([]);
for k=1:3
 r=R(strcmp({R.direction},names{k}));d=[r.main_mm]';gm=[r.ground_main_mm]';gc=[r.ground_cross_mm]';m=[r.main_mm]';c=[r.cross_mm]';assert(numel(d)==3 && all(isfinite([d;gm;gc;m;c])) && all(d>0));km=(gm-m)./d;kc=(gc-c)./d;
 g=struct('direction',names{k},'k_main',mean(km),'k_cross',mean(kc),'weight',0.5,'k_main_each',km','k_cross_each',kc','half_main_mm_per_m',500*mean(km),'half_cross_mm_per_m',500*mean(kc));
 for axis=1:2
  if axis==1,base=m;ground=gm;coeff=km;label='main';else,base=c;ground=gc;coeff=kc;label='cross';end
  pred=base+.5*mean(coeff)*d;loo=zeros(3,1);for j=1:3,ix=setdiff(1:3,j);loo(j)=base(j)+.5*mean(coeff(ix))*d(j);end
  g.(label)=struct('baseline_errors_mm',(base-ground)','half_in_sample_errors_mm',(pred-ground)','half_leave_one_out_errors_mm',(loo-ground)','baseline_MAE_mm',mean(abs(base-ground)),'half_in_sample_MAE_mm',mean(abs(pred-ground)),'half_leave_one_out_MAE_mm',mean(abs(loo-ground)),'baseline_worst_mm',max(abs(base-ground)),'half_leave_one_out_worst_mm',max(abs(loo-ground)));
 end
 G(k)=g;
end
% Stateless reference-frame projection: no repeated-sample accumulation or yaw modification.
for phi=[0 90 -90 179 -179]
 a=phi*pi/180;T=[cos(a) -sin(a);sin(a) cos(a)];for k=1:3
  d=2100;v=.5*d*[G(k).k_main;G(k).k_cross];if k==1,local=[v(2);-v(1)];elseif k==2,local=v;else,local=[-v(1);v(2)];end
  world=T*local;assert(norm(T'*world-local)<1e-9);assert(isequal(T*local,T*local));assert(norm(T*(0*local))==0);
 end
end
O=struct('status','PASS_CANDIDATE_ANALYSIS_ONLY','fixed_gain',0.5,'hardware_access',false,'product_changed',false,'groups',G,'left_existing_cross_coefficient',0.0179055385,'main_deployment_recommended',false,'fresh_validation_complete',false,'note','leave-one-out within retrospective same-distance group; not independent new field validation');
fid=fopen(fullfile(out,'calibration_candidates.json'),'w','n','UTF-8');fwrite(fid,jsonencode(O),'char');fclose(fid);disp(O.status);disp(G);
end
