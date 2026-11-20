function validate_left_nav_residual
% Diagnostic model only. Raw HWT-projected wheel coordinates are immutable.
out=fileparts(mfilename('fullpath'));diary(fullfile(out,'matlab_diary.log'));
inputs=jsondecode(fileread(fullfile(out,'inputs.json')));records=inputs.records;
train=zeros(3,8);traces=cell(6,1);models=cell(6,1);grounds=cell(6,1);
for n=1:3
 assert(strcmp(records(n).group,'train'));p=jsondecode(fileread(records(n).prediction));g=jsondecode(fileread(records(n).measurement));T=readtable(records(n).trace);
 assert(all(isfinite(T.path_F_mm))&&all(isfinite(T.path_L_mm)));
 assert(abs(T.path_F_mm(end)-p.cross_forward_mm)<1e-8 && abs(T.path_L_mm(end)-p.main_left_mm)<1e-8);
 assert(abs(p.cross_forward_mm-p.body_F_mm-p.yaw_effect_cross_mm)<1e-8);
 assert(p.main_left_mm>2000 && p.main_left_mm<2200);
 residual=g.forward_mm-p.cross_forward_mm;
 % HWT is already included in modelF. This decomposition subtracts it once.
 assert(abs(residual-(g.forward_mm-p.body_F_mm-p.yaw_effect_cross_mm))<1e-8);
 train(n,:)=[n,p.main_left_mm,p.cross_forward_mm,g.forward_mm,p.body_F_mm,p.yaw_effect_cross_mm,residual,residual/p.main_left_mm];
 traces{n}=T;models{n}=p;grounds{n}=g;
end
k=mean(train(:,8));assert(k>0 && k<.03);
freeze=struct('status','FROZEN_THREE_RUN_MEAN_BEFORE_HELDOUT_SCORING','coefficient',k,...
 'training_runs',[1 2 3],'definition','mean((groundF-modelF)/modelL)',...
 'no_main_scale_gamma',true,'HWT_already_in_modelF',true,'heldout_used_for_fit',false,...
 'provisional_lambda',.5,'lambda_is_engineering_gain_not_confidence',true);
write_json(fullfile(out,'frozen_candidate.json'),freeze);
% Left150 is loaded only after k is frozen. No heldout fitting or run exclusion.
for n=4:6
 assert(strcmp(records(n).group,'heldout'));p=jsondecode(fileread(records(n).prediction));g=jsondecode(fileread(records(n).measurement));T=readtable(records(n).trace);
 assert(all(isfinite(T.path_F_mm))&&all(isfinite(T.path_L_mm)));
 assert(abs(T.path_F_mm(end)-p.cross_forward_mm)<1e-8 && abs(T.path_L_mm(end)-p.main_left_mm)<1e-8);
 traces{n}=T;models{n}=p;grounds{n}=g;
end
score=zeros(18,12);ix=0;maxProjectionError=0;maxSingleError=0;
for n=1:6
 T=traces{n};p=models{n};g=grounds{n};
 for lambda=[0 .5 1]
  correctedF=T.path_F_mm+lambda*k*T.path_L_mm;
  correctedL=T.path_L_mm;
  assert(isequal(correctedL,T.path_L_mm));
  if lambda==0,assert(isequal(correctedF,T.path_F_mm));end
  correction=correctedF-T.path_F_mm;
  assert(max(abs(correction-lambda*k*T.path_L_mm))<1e-10);
  if lambda==.5,assert(max(abs(correction-.5*(k*T.path_L_mm)))<1e-10);end
  % Rotate the same raw path into nonzero global reference frames.
  for psi=[-179.8 -90 37 179.8 180.2 359.9]
   c=cosd(psi);s=sind(psi);origin=[2100 -700];
   rawX=origin(1)+c*T.path_F_mm-s*T.path_L_mm;
   rawY=origin(2)+s*T.path_F_mm+c*T.path_L_mm;
   left= -s*(rawX-origin(1))+c*(rawY-origin(2));
   navX=rawX+c*lambda*k*left;navY=rawY+s*lambda*k*left;
   F=c*(navX-origin(1))+s*(navY-origin(2));L=-s*(navX-origin(1))+c*(navY-origin(2));
   err=max([max(abs(F-correctedF)),max(abs(L-correctedL))]);maxProjectionError=max(maxProjectionError,err);assert(err<1e-8);
   % C-like float publication adds only the small residual; no total scaling.
   fc=single(c);fs=single(s);rx=single(rawX);ry=single(rawY);ox=single(origin(1));oy=single(origin(2));
   ll=-fs*(rx-ox)+fc*(ry-oy);dd=single(lambda)*single(k)*ll;
   nx=rx+fc*dd;ny=ry+fs*dd;
   serr=max([max(abs(double(nx)-navX)),max(abs(double(ny)-navY))]);maxSingleError=max(maxSingleError,serr);assert(serr<.002);
  end
  ix=ix+1;score(ix,:)=[n,n<=3,lambda,p.main_left_mm,p.cross_forward_mm,g.actual_left_mm,g.forward_mm,...
   p.main_left_mm-g.actual_left_mm,p.cross_forward_mm-g.forward_mm,correctedF(end),correctedF(end)-g.forward_mm,lambda*k*p.main_left_mm];
 end
end
% Provisional one-left-reference lifecycle contract. No mutable corrected input.
% A new reference resets origin/cache; invalid freezes numeric output but valid0.
% Non-left/disabled output raw, so scope changes require an explicit new frame.
s=struct('seen',false,'ref',0,'sample',0,'nav',[0 0],'valid',false);
in=struct('reference',10,'sample',1,'origin',[200 -300],'psi',179.8,'raw',[200 -300],...
 'valid',true,'left_scope',true,'enabled',true);[s,v]=step(s,in,k,.5);assert(isequal(v.xy,in.raw));
for psi=[37 179.8 -179.8]
 in.reference=in.reference+1;in.sample=1;in.psi=psi;in.raw=in.origin;
 [s,v]=step(s,in,k,.5);assert(isequal(v.xy,in.raw));
 in.sample=2;in.raw=in.origin+[-sind(psi) cosd(psi)]*1500+[cosd(psi) sind(psi)]*(-30);
 [s,v]=step(s,in,k,.5);saved=v.xy;[s,v2]=step(s,in,k,.5);assert(isequal(saved,v2.xy));
 % Invalid/raw-changing data must not advance corrected position or freshness.
 in.sample=3;in.valid=false;in.raw=in.raw+[999 999];[s,bad]=step(s,in,k,.5);assert(isequal(bad.xy,saved)&&~bad.valid&&s.sample==2);
 in.valid=true;in.sample=4;in.raw=in.origin;in.reference=in.reference+1;[s,fresh]=step(s,in,k,.5);assert(isequal(fresh.xy,in.raw));
 in.enabled=false;in.sample=5;in.raw=in.origin+[100 20];[s,off]=step(s,in,k,.5);assert(isequal(off.xy,in.raw));
 in.enabled=true;in.left_scope=false;in.sample=6;[s,nonleft]=step(s,in,k,.5);assert(isequal(nonleft.xy,in.raw));in.left_scope=true;
end
% Crossing +/-180 actual HWT is already represented by raw rotation: no
% secondary commanded-angle/heading correction is added by this residual map.
a=[179.8 179.9 -179.99 -179.8];unwrapped=unwrap(a*pi/180);assert(all(diff(unwrapped)>0));
dleft=[10 15 20];halfyaw=diff(unwrapped)/2;midyaw=(unwrapped(1:end-1)+unwrapped(2:end))/2;
arc=sin(halfyaw)./halfyaw;rawGlobal=[0 0;cumsum([-arc.*dleft.*sin(midyaw);arc.*dleft.*cos(midyaw)]',1)];
ref=179.8;c=cosd(ref);ss=sind(ref);rawF=c*rawGlobal(:,1)+ss*rawGlobal(:,2);rawL=-ss*rawGlobal(:,1)+c*rawGlobal(:,2);
assert(abs(rawF(end))>.01);
correctedGlobal=rawGlobal+[c*.5*k*rawL,ss*.5*k*rawL];
assert(max(abs(c*correctedGlobal(:,1)+ss*correctedGlobal(:,2)-(rawF+.5*k*rawL)))<1e-10);
assert(max(abs(-ss*correctedGlobal(:,1)+c*correctedGlobal(:,2)-rawL))<1e-10);
assert(isequal(a,[179.8 179.9 -179.99 -179.8]));
raw=[1 2;3 4];copy=raw;nav=raw+[k*10 0;k*20 0];assert(isequal(raw,copy));assert(~isequal(raw,nav));
metrics=zeros(2,7);
for group=1:2
 mask=(score(:,2)==(group==1));
 metrics(group,1)=group;
 for j=1:3
  lambda=[0 .5 1];rows=score(mask & score(:,3)==lambda(j),:);
  metrics(group,1+j)=mean(abs(rows(:,11)));metrics(group,4+j)=max(abs(rows(:,11)));
 end
end
% Fail if any trained/heldout metadata accidentally applies gamma to left.
assert(all(score(:,4)-score(:,6)==score(:,8)));
summary=struct('status','PASS_DIAGNOSTIC_PROVISIONAL_LIFECYCLE','matlab_version',version,...
 'coefficient',k,'per_run_coefficients',train(:,8)','training_count',3,'heldout_count',3,...
 'projection_max_error_mm',maxProjectionError,'float_publish_max_error_mm',maxSingleError,...
 'lambda_sensitivity',[0 .5 1],'training_cross_MAE_mm',metrics(1,2:4),...
 'heldout_cross_MAE_mm',metrics(2,2:4),'heldout_cross_worst_mm',metrics(2,5:7),...
 'no_main_gamma',true,'hardware_accessed',false,'product_written',false,...
 'limitations',{{'Residual is apparent end-window error, not independently isolated wheel slip',...
 'Ground ruler and model source window differ; no synchronized ground trajectory',...
 'Left150 data already observed; retrospective heldout scoring, not prospective proof',...
 '50percent gain scales only residual; it is not a statistical confidence',...
 'Single-left reference lifecycle is provisional pending xhigh/root contract',...
 'Never pass corrected pose through raw getter or apply again to ICM wheel anchors'}});
writematrix(train,fullfile(out,'training_decomposition.csv'));writematrix(score,fullfile(out,'sensitivity_scores.csv'));
writematrix(metrics,fullfile(out,'sensitivity_metrics.csv'));write_json(fullfile(out,'matlab_summary.json'),summary);disp(summary);diary off;
end
function [state,out]=step(state,in,k,lambda)
if ~in.valid || ~all(isfinite([in.raw in.origin in.psi k lambda])),out=struct('xy',state.nav,'valid',false);state.valid=false;return;end
if state.seen && state.ref==in.reference && state.sample==in.sample
 out=struct('xy',state.nav,'valid',state.valid);return;
end
if ~state.seen || state.ref~=in.reference,state.nav=in.raw;state.sample=0;end
xy=in.raw;
if in.enabled && in.left_scope
 L=-sind(in.psi)*(in.raw(1)-in.origin(1))+cosd(in.psi)*(in.raw(2)-in.origin(2));
 xy=xy+[cosd(in.psi) sind(in.psi)]*(lambda*k*L);
end
state.seen=true;state.ref=in.reference;state.sample=in.sample;state.nav=xy;state.valid=true;
out=struct('xy',xy,'valid',true);
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(value));fclose(fid);
end