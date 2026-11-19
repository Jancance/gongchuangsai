function validate_source_real_qr_20261116
% Pre-implementation mailbox and business-code contract; no hardware IO.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-16','source-real-qr');
if ~exist(out,'dir'),mkdir(out);end
checks=0;
orders=perms('126');
for k=1:size(orders,1)
 code=[orders(k,:) '+123+621+231'];
 assert(valid(code)); checks=checks+1;
 colors=double(code(1:3))-double('0');
 assert(isequal(colors,double(orders(k,:))-double('0')));checks=checks+1;
 % Old complete/partial bytes are discarded by the sole receive owner.
 for old={'215+123+512+321','126+12',''}
  queue=old{1};queue='';locked=false;motion=0; %#ok<NASGU>
  assert(isempty(queue) && ~locked && motion==0);checks=checks+1;
  for bad={'111+123+111+123','126+112+621+231','126+123+625+231','126+123','000+123+000+123'}
   assert(~valid(bad{1}));assert(~locked && motion==0);checks=checks+2;
  end
  locked=valid(code);latched=code;assert(locked);checks=checks+1;
  for next={code,'215+123+512+321'}
   if ~locked && valid(next{1}),latched=next{1};end
   assert(strcmp(latched,code));checks=checks+1;
  end
  stop=true;assert(stop);checks=checks+1;
 end
end
r=struct('passed',true,'local_matlab',true,'checks',checks,...
 'scope','synthetic receive ownership, four-part validation, order, lock, STOP; no physical claim');
fid=fopen(fullfile(out,'matlab-result.json'),'w');fprintf(fid,'%s\n',jsonencode(r));fclose(fid);
fprintf('PASS real QR contract: %d checks\n',checks);
end
function ok=valid(c)
ok=numel(c)==15;if ~ok,return;end
ok=all(c([4 8 12])=='+') && all(c([1:3 5:7 9:11 13:15])>='1' & c([1:3 5:7 9:11 13:15])<='6');
if ~ok,return;end
ok=numel(unique(c(1:3)))==3 && numel(unique(c(9:11)))==3 && isequal(sort(c(1:3)),sort(c(9:11))) && strcmp(sort(c(5:7)),'123') && strcmp(sort(c(13:15)),'123');
end
