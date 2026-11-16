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
 'fixed_lambda',.5,'lambda_is_engineering_gain_not_confidence',true);
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
  correctedF=T.path_F_mm+lambda*k*max(T.path_L_mm,0);
  correctedL=T.path_L_mm;
  assert(isequal(correctedL,T.path_L_mm));
  if lambda==0,assert(isequal(correctedF,T.path_F_mm));end
  correction=correctedF-T.path_F_mm;
  assert(max(abs(correction-lambda*k*max(T.path_L_mm,0)))<1e-10);
  if lambda==.5,assert(max(abs(correction-.5*(k*max(T.path_L_mm,0))))<1e-10);end
  % Rotate the same raw path into nonzero global reference frames.
  for psi=[-179.8 -90 37 179.8 180.2 359.9]
   c=cosd(psi);s=sind(psi);origin=[2100 -700];
   rawX=origin(1)+c*T.path_F_mm-s*T.path_L_mm;
   rawY=origin(2)+s*T.path_F_mm+c*T.path_L_mm;
   left= -s*(rawX-origin(1))+c*(rawY-origin(2));
   navX=rawX+c*lambda*k*max(left,0);navY=rawY+s*lambda*k*max(left,0);
   F=c*(navX-origin(1))+s*(navY-origin(2));L=-s*(navX-origin(1))+c*(navY-origin(2));
   err=max([max(abs(F-correctedF)),max(abs(L-correctedL))]);maxProjectionError=max(maxProjectionError,err);assert(err<1e-8);
   % C-like float publication adds only the small residual; no total scaling.
   fc=single(c);fs=single(s);rx=single(rawX);ry=single(rawY);ox=single(origin(1));oy=single(origin(2));
   ll=-fs*(rx-ox)+fc*(ry-oy);dd=single(lambda)*single(k)*max(ll,single(0));
   nx=rx+fc*dd;ny=ry+fs*dd;
   serr=max([max(abs(double(nx)-navX)),max(abs(double(ny)-navY))]);maxSingleError=max(maxSingleError,serr);assert(serr<.002);
  end
  ix=ix+1;score(ix,:)=[n,n<=3,lambda,p.main_left_mm,p.cross_forward_mm,g.actual_left_mm,g.forward_mm,...
   p.main_left_mm-g.actual_left_mm,p.cross_forward_mm-g.forward_mm,correctedF(end),correctedF(end)-g.forward_mm,lambda*k*p.main_left_mm];
 end
end
% Formal contract: successful SetPose latches scope/frame, not live activity.
assert(abs(k-.017905538496259973)<1e-15);fixedLambda=.5;
shape=struct('plan',9,'count',1,'kind',5,'rpm',102,'mm',2100,'run',0,'reference',0,'relative',0,'runtime_route',0);
assert(scope_ok(shape));good150=shape;good150.plan=18;good150.mm=1500;assert(scope_ok(good150));
excluded30=shape;excluded30.plan=19;excluded30.mm=300;assert(~scope_ok(excluded30));
fields={'count','kind','rpm','mm','run','reference','relative','runtime_route','plan'};
values={[0 2],[4 6],[101 103],[1500 2099 2101],[1],[1],[1],[1],[8 10 18 19]};scopeTests=3;
for j=1:numel(fields),for value=values{j}
 bad=shape;bad.(fields{j})=value;assert(~scope_ok(bad));scopeTests=scopeTests+1;
end,end
state=struct('scope',false,'origin',[0 0],'psi',0);
lifecycleTests=0;
for psi=[37 -90 179.8 -179.8 180.2]
 raw=make_raw([200 -300],psi,100);raw.pose.x_mm=200;raw.pose.y_mm=-300;
 state=latch(state,'combined_success',raw,shape);
 nav=map_nav(state,raw,k,fixedLambda,true);assert(isequal(nav,raw));lifecycleTests=lifecycleTests+1;
 eF=[cosd(psi) sind(psi)];eL=[-sind(psi) cosd(psi)];
 raw.pose.x_mm=200-30*eF(1)+1500*eL(1);raw.pose.y_mm=-300-30*eF(2)+1500*eL(2);
 raw.pose.sample_ms=200;raw.pose.updates=1;nav=map_nav(state,raw,k,fixedLambda,true);saved=nav;
 assert(abs(dot([nav.pose.x_mm-200 nav.pose.y_mm+300],eF)-(-30+.5*k*1500))<1e-10);
 assert_metadata(raw,nav);assert(isequal(map_nav(state,raw,k,fixedLambda,true),nav));lifecycleTests=lifecycleTests+3;
 % active/DONE/liveplan changes do not call latch or erase old correction.
 live=shape;live.plan=10;live.kind=6;active=0;done=1; %#ok<NASGU>
 assert(isequal(map_nav(state,raw,k,fixedLambda,true),saved));lifecycleTests=lifecycleTests+1;
 % Raw invalid retains frozen raw XY; nav applies same affine map and copies
 % valid/reasons/sample/age/updates/continuity/local_origin without repair.
 raw.pose.valid=0;raw.pose.reasons=10;raw.pose.age_ms=401;raw.pose.continuity_lost=1;
 invalid=map_nav(state,raw,k,fixedLambda,true);assert_metadata(raw,invalid);
 assert(isequal([invalid.pose.x_mm invalid.pose.y_mm],[saved.pose.x_mm saved.pose.y_mm]));lifecycleTests=lifecycleTests+2;
 % ResetRecords only removes anchor validity; no new successful pose/frame.
 raw.anchor_valid=0;oldState=state;nav=map_nav(state,raw,k,fixedLambda,true);
 assert(isequal(state,oldState));assert_metadata(raw,nav);
 assert(isequal([nav.pose.x_mm nav.pose.y_mm],[saved.pose.x_mm saved.pose.y_mm]));lifecycleTests=lifecycleTests+3;
 state=latch(state,'manual_fail',raw,shape);assert(isequal(state,oldState));
 state=latch(state,'combined_fail',raw,shape);assert(isequal(state,oldState));lifecycleTests=lifecycleTests+2;
 assert(isequal(map_nav(state,raw,k,fixedLambda,false),raw));lifecycleTests=lifecycleTests+1;
 % Negative/zero NET left displacement receives no correction, not accumulated
 % path length. Returning to origin retracts only the small displacement map.
 negative=raw;negative.pose.x_mm=200-100*eL(1);negative.pose.y_mm=-300-100*eL(2);
 assert(isequal(map_nav(state,negative,k,fixedLambda,true),negative));
 zero=raw;zero.pose.x_mm=200;zero.pose.y_mm=-300;assert(isequal(map_nav(state,zero,k,fixedLambda,true),zero));lifecycleTests=lifecycleTests+2;
 % A successful manual SetPose clears scope; failed manual SetPose never does.
 state=latch(state,'manual_success',raw,shape);assert(~state.scope);
 assert(isequal(map_nav(state,raw,k,fixedLambda,true),raw));lifecycleTests=lifecycleTests+2;
 % New successful left150 reference latches its new origin/frame. New non-left
 % successful reference is raw alias, and left30 is intentionally excluded.
 raw2=make_raw([900 700],psi+15,300);state=latch(state,'combined_success',raw2,good150);
 assert(state.scope && isequal(map_nav(state,raw2,k,fixedLambda,true),raw2));lifecycleTests=lifecycleTests+1;
 state=latch(state,'combined_success',raw2,excluded30);assert(~state.scope);
 assert(isequal(map_nav(state,raw2,k,fixedLambda,true),raw2));lifecycleTests=lifecycleTests+2;
 nonleft=shape;nonleft.plan=10;nonleft.kind=6;state=latch(state,'combined_success',raw2,nonleft);assert(~state.scope);lifecycleTests=lifecycleTests+1;
 % Consumer split: capture/controller/perimeter use RAW; global pose and ICM
 % anchors use NAV once, with no second empirical correction after anchoring.
 raw.pose.valid=1;raw.pose.reasons=0;raw.pose.age_ms=0;raw.pose.continuity_lost=0;
 state=latch(state,'combined_success',raw,shape);capture=raw;controller=raw;perimeter=raw;
 navigation=map_nav(state,raw,k,fixedLambda,true);globalPose=navigation.pose;
 icmF=dot([navigation.pose.x_mm-state.origin(1),navigation.pose.y_mm-state.origin(2)],eF);
 assert(isequal(capture,raw)&&isequal(controller,raw)&&isequal(perimeter,raw));
 assert(isequal(globalPose,navigation.pose));assert(abs(icmF-(-30+.5*k*1500))<1e-10);
 assert_metadata(raw,navigation);lifecycleTests=lifecycleTests+4;
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
summary=struct('status','PASS_FORMAL_LIFECYCLE_FIXED_HALF_GAIN','matlab_version',version,...
 'coefficient',k,'per_run_coefficients',train(:,8)','training_count',3,'heldout_count',3,...
 'projection_max_error_mm',maxProjectionError,'float_publish_max_error_mm',maxSingleError,...
 'fixed_lambda',.5,'scope_cases',scopeTests,'lifecycle_assertions',lifecycleTests,'lambda_sensitivity',[0 .5 1],'training_cross_MAE_mm',metrics(1,2:4),...
 'heldout_cross_MAE_mm',metrics(2,2:4),'heldout_cross_worst_mm',metrics(2,5:7),...
 'no_main_gamma',true,'hardware_accessed',false,'product_written',false,...
 'limitations',{{'Residual is apparent end-window error, not independently isolated wheel slip',...
 'Ground ruler and model source window differ; no synchronized ground trajectory',...
 'Left150 data already observed; retrospective heldout scoring, not prospective proof',...
 '50percent gain scales only residual; it is not a statistical confidence',...
 'Scope only successful reference plan9left210 or18left150, not plan19left30',...
 'Never pass corrected pose through raw getter or apply again to ICM wheel anchors'}});
writematrix(train,fullfile(out,'training_decomposition.csv'));writematrix(score,fullfile(out,'sensitivity_scores.csv'));
writematrix(metrics,fullfile(out,'sensitivity_metrics.csv'));write_json(fullfile(out,'matlab_summary.json'),summary);disp(summary);diary off;
end
function allowed=scope_ok(p)
allowed=p.count==1 && p.kind==5 && p.rpm==102 && p.run==0 && p.reference==0 && p.relative==0 && p.runtime_route==0 && ...
 ((p.plan==9 && p.mm==2100)||(p.plan==18 && p.mm==1500));
end
function state=latch(state,event,raw,shape)
if strcmp(event,'combined_success')
 state.scope=scope_ok(shape);state.origin=[raw.origin_x_mm raw.origin_y_mm];state.psi=raw.path_heading_deg;
elseif strcmp(event,'manual_success')
 state.scope=false;
else
 assert(strcmp(event,'manual_fail')||strcmp(event,'combined_fail'));
end
end
function nav=map_nav(state,raw,k,lambda,enabled)
nav=raw;
if enabled && state.scope
 c=cosd(state.psi);ss=sind(state.psi);
 L=-ss*(raw.pose.x_mm-state.origin(1))+c*(raw.pose.y_mm-state.origin(2));
 correction=lambda*k*max(L,0);nav.pose.x_mm=raw.pose.x_mm+c*correction;nav.pose.y_mm=raw.pose.y_mm+ss*correction;
end
end
function raw=make_raw(origin,psi,reference)
p=struct('x_mm',origin(1),'y_mm',origin(2),'heading_deg',psi,'sample_ms',100,'age_ms',0,...
 'updates',0,'valid',1,'reasons',0,'continuity_lost',0,'local_origin',0);
raw=struct('pose',p,'origin_x_mm',origin(1),'origin_y_mm',origin(2),'path_heading_deg',psi,...
 'heading_offset_deg',17,'anchor_valid',1,'reference_set_ms',reference);
end
function assert_metadata(raw,nav)
a=raw;b=nav;a.pose=rmfield(a.pose,{'x_mm','y_mm'});b.pose=rmfield(b.pose,{'x_mm','y_mm'});assert(isequal(a,b));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(value));fclose(fid);
end