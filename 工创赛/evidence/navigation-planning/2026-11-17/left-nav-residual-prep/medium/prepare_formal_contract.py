from pathlib import Path
import shutil
E=Path('D:/Desktop/工创赛/工创赛/evidence/navigation-planning/2026-11-17/left-nav-residual-prep/medium');archive=E/'provisional_lifecycle_before_contract';archive.mkdir(exist_ok=True)
for n in ['validate_left_nav_residual.m','matlab_summary.json','matlab_receipt.json','matlab_process.log','matlab_diary.log','frozen_candidate.json','REPORT.md','training_decomposition.csv','sensitivity_scores.csv','sensitivity_metrics.csv']:
 shutil.copyfile(E/n,archive/n)
p=E/'validate_left_nav_residual.m';s=p.read_text('utf8')
s=s.replace('correctedF=T.path_F_mm+lambda*k*T.path_L_mm;','correctedF=T.path_F_mm+lambda*k*max(T.path_L_mm,0);')
s=s.replace('correction-lambda*k*T.path_L_mm','correction-lambda*k*max(T.path_L_mm,0)').replace('correction-.5*(k*T.path_L_mm)','correction-.5*(k*max(T.path_L_mm,0))')
s=s.replace('navX=rawX+c*lambda*k*left;navY=rawY+s*lambda*k*left;','navX=rawX+c*lambda*k*max(left,0);navY=rawY+s*lambda*k*max(left,0);')
s=s.replace('dd=single(lambda)*single(k)*ll;','dd=single(lambda)*single(k)*max(ll,single(0));')
a=s.index('% Provisional one-left-reference lifecycle contract.');b=s.index('% Crossing +/-180 actual HWT',a)
lifecycle=r'''% Formal contract: successful SetPose latches scope/frame, not live activity.
assert(abs(k-.017905538496259973)<1e-15);fixedLambda=.5;
shape=struct('plan',9,'count',1,'kind',5,'rpm',102,'mm',2100,'run',0,'reference',0,'relative',0,'runtime_route',0);
assert(scope_ok(shape));good150=shape;good150.plan=18;good150.mm=1500;assert(scope_ok(good150));
bad=shape;bad.plan=19;bad.mm=300;assert(~scope_ok(bad));
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
 state=latch(state,'combined_success',raw2,bad);assert(~state.scope);
 assert(isequal(map_nav(state,raw2,k,fixedLambda,true),raw2));lifecycleTests=lifecycleTests+2;
 % Consumer split: capture/controller/perimeter use RAW; global pose and ICM
 % anchors use NAV once, with no second empirical correction after anchoring.
 state=latch(state,'combined_success',raw,shape);capture=raw;controller=raw;perimeter=raw;
 navigation=map_nav(state,raw,k,fixedLambda,true);globalPose=navigation.pose;
 icmF=dot([navigation.pose.x_mm-state.origin(1),navigation.pose.y_mm-state.origin(2)],eF);
 assert(isequal(capture,raw)&&isequal(controller,raw)&&isequal(perimeter,raw));
 assert(isequal(globalPose,navigation.pose));assert(abs(icmF-(-30+.5*k*1500))<1e-10);
 assert_metadata(raw,navigation);lifecycleTests=lifecycleTests+4;
end
'''
s=s[:a]+lifecycle+s[b:]
s=s.replace("'PASS_DIAGNOSTIC_PROVISIONAL_LIFECYCLE'","'PASS_FORMAL_LIFECYCLE_FIXED_HALF_GAIN'")
s=s.replace("'lambda_sensitivity',[0 .5 1],","'fixed_lambda',.5,'scope_cases',scopeTests,'lifecycle_assertions',lifecycleTests,'lambda_sensitivity',[0 .5 1],")
s=s.replace("'Single-left reference lifecycle is provisional pending xhigh/root contract',...","'Scope only successful reference plan9left210 or18left150, not plan19left30',...")
a=s.index('function [state,out]=step(');b=s.index('function write_json(',a)
helpers=r'''function allowed=scope_ok(p)
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
'''
s=s[:a]+helpers+s[b:];p.write_text(s,'utf8');print('Formal lifecycle MATLAB patched; provisional scripts and outputs archived; product untouched')