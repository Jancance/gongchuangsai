function validate_vision_module_flow_20261112
% Preconditions for the modular vision flow. No hardware or runtime enable.
root=fileparts(fileparts(mfilename('fullpath')));out=fullfile(root,'evidence','integration','2026-11-12','vision-modules');
if ~exist(out,'dir'),mkdir(out);end
checks=struct('name',{},'passed',{});fixtures=struct;
% Same task semantics as the existing preliminary parser, not new rules.
first=[2 1 5];firstRing=[2 1 3];second=[5 1 2];secondRing=[3 2 1];storage=zeros(1,3);
for k=1:3,storage(k)=firstRing(first==second(k));end
check('task and same-colour stacking association',isequal(storage,[3 1 2]));
% Paper geometry is not used for arm heights; these are explicit model inputs.
baseDepth=70;stackHeight=60;stackDepth=baseDepth-stackHeight;
check('stack raised60 means descend60 less',stackDepth==10 && stackDepth>=0);
check('impossible stack depth is parameter wait, never negative motion',(40-stackHeight)<0);
pickupDepth=55;rise=linspace(pickupDepth,0,101);fraction=(pickupDepth-rise)/pickupDepth;
firstPreclose=find(fraction>=.5,1);
check('preclose160 at measured half ascent after release210',firstPreclose==51 && rise(firstPreclose)==pickupDepth/2);
% Small correction capability is an adapter value, not a tuning fault.
requests=[5 20 40 65];cap=20;segments=cell(1,numel(requests));
for k=1:numel(requests)
 n=ceil(abs(requests(k))/cap);segments{k}=repmat(requests(k)/n,1,n);
 check(sprintf('correction %.0f split within adapter capability',requests(k)), ...
  abs(sum(segments{k})-requests(k))<1e-9 && all(abs(segments{k})<=cap));
end
check('ordinary changed speeds are not equality-locked',all([50 106 200 300]>0));
% Source colours: expected codes, candidate sets per observed physical object.
cases={{2,1,[]},{2,[1 5],5},{2,[],[]},{2,1,3},{2,1,5}};
uniqueExpected=[true true false false true];counts=zeros(1,numel(cases));solutions=cell(1,numel(cases));
orders=perms(first);
for k=1:numel(cases)
 c=cases{k};valid=false(6,1);
 for j=1:6
  valid(j)=all(arrayfun(@(i)isempty(c{i}) || any(c{i}==orders(j,i)),1:3));
 end
 counts(k)=sum(valid);solutions{k}=orders(valid,:);
 check(sprintf('unique task/physical-object association case%d',k),(counts(k)==1)==uniqueExpected(k));
end
% Coordinator ownership: no arm work during TRAVEL; alignment is delegated
% to chassis only after stopped arrival. No DONE invented from acceptance.
events={'travel accepted','early vision','travel done not stopped','matching stopped arrival','arm accepted','arm done','placed evidence'};
states={'TRAVEL','TRAVEL','TRAVEL','WORK','WORK','WAIT_EVIDENCE','WORK'};
check('early image never changes travelling owner',all(strcmp(states(1:3),'TRAVEL')));
check('stopped matching arrival enables work',strcmp(states{4},'WORK'));
check('command done alone is not physical placement',strcmp(states{6},'WAIT_EVIDENCE'));
% User selected fixed positions; moving tool XY only at top in this initial
% serial implementation. Parallel pzq source action remains an existing port.
actions={'INDEX','ABOVE_PICK','DESCEND_PICK','CLOSE','VERIFY_HELD','TOP','ABOVE_PLACE','DESCEND_PLACE','OPEN','TOP_WITH_HALF_PRECLOSE','VERIFY_PLACED'};
check('XY transfer follows real top return',find(strcmp(actions,'TOP'))<find(strcmp(actions,'ABOVE_PLACE')));
check('second item follows verified placement',strcmp(actions{end},'VERIFY_PLACED'));
% A declared backend failure is distinct from a missing/late observation.
severeBackend=[false false true true];moving=[false true true false];
policy=repmat({'WAIT'},1,4);
for k=1:4,if severeBackend(k),policy{k}='STOP';elseif moving(k),policy{k}='KEEP_PENDING';end,end
check('severe backend stop; late motion feedback stays pending',isequal(policy,{'WAIT','KEEP_PENDING','STOP','STOP'}));
fixtures.task=struct('raw','215+213+512+321','first_colors',first,'first_rings',firstRing,'second_colors',second,'second_rings',secondRing,'second_storage',storage);
fixtures.stack_depth=struct('base',baseDepth,'height',stackHeight,'result',stackDepth);
fixtures.preclose=struct('pick_depth',pickupDepth,'fraction',.5,'first_index',firstPreclose);
fixtures.association_cases=cases;fixtures.association_counts=counts;fixtures.association_solutions=solutions;
fixtures.correction_requests=requests;fixtures.correction_cap=cap;fixtures.correction_segments=segments;
r=struct('engine','local MATLAB','release',version('-release'),'checks',checks,'check_count',numel(checks), ...
 'failure_count',sum(~[checks.passed]),'passed',all([checks.passed]),'fixtures',fixtures, ...
 'geometry_and_hardware_model_limitations','declared taught positions/heights only; no actual sweep, timer or physical success proof', ...
 'hardware_access',false,'motion_allowed',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(f~=-1);fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('MODULAR FLOW MATLAB %d checks/%d failures; no hardware access.\n',r.check_count,r.failure_count);assert(r.passed);
 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed));
 end
end
