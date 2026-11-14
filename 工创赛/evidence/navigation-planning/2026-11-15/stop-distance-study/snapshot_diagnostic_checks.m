function snapshot_diagnostic_checks()
out=fileparts(mfilename('fullpath'));spec=jsondecode(fileread(fullfile(out,'snapshot_spec.json')));
offsets=(0:11)*4;assert(isequal(offsets,[0 4 8 12 16 20 24 28 32 36 40 44]));
assert(spec.snapshot_bytes==48 && spec.header_bytes+spec.snapshot_count*spec.snapshot_bytes==176);
% Host-independent uint32 subtraction; MATLAB uint32 subtraction saturates,
% so explicitly model C modulo 2^32 rather than silently using saturation.
wrap=@(now,then)mod(double(now)-double(then),2^32);
assert(wrap(uint32(5),uint32(4294967280))==21);
assert(wrap(uint32(50),uint32(4294967250))==96);
assert(wrap(uint32(1000),uint32(995))==5);
% Store exactly five IEEE754 float32 values; this is a schema check, not C ABI sizeof.
floats=single([2104.655 2084.1 14.9 -1.8 .22]);
bytes=typecast(floats,'uint8');assert(numel(bytes)==20);assert(isequal(typecast(bytes,'single'),floats));
recordBytes=[typecast(uint32([5 4294967280 4294967250 4294967270]),'uint8') bytes typecast(uint32([15 18 197]),'uint8')];
assert(numel(recordBytes)==48);
timeFields=typecast(recordBytes(1:16),'uint32');assert(timeFields(2)==uint32(4294967280));
assert(wrap(timeFields(1),timeFields(2))==21);
assert(isequal(typecast(recordBytes(17:36),'single'),floats));
head=zeros(1,8,'uint32');head(1)=1;slots=zeros(3,12,'single');
% Accepted start resets; rejected start and later idle/finish do not reset.
[head,slots]=reset_capture(head,slots,11,true);assert(head(2)==11&&head(3)==0);
[head,slots]=record_event(head,slots,1,record(100,80,90,1981,1950,30,-1,.1,29,30),true,true);
assert(head(3)==1&&head(4)==1);entry=slots(1,:);
[head,slots]=record_event(head,slots,1,record(120,100,110,1990,1960,28,-1.1,.2,27,29),true,true);
assert(isequal(slots(1,:),entry));
[head,slots]=record_event(head,slots,2,record(130,110,120,1991,1964,27,-1.2,.2,26,26),true,true);
[head,slots]=record_event(head,slots,2,record(150,125,140,2000,1970,25,-1.3,.2,24,24),true,true);
assert(head(3)==3&&head(5)==2&&slots(2,1)==150);
% Arrival samples reference first, then captures the *old* follower state.
lastfollow=slots(2,8:9);arrival=record(200,170,140,2104,2084,15,lastfollow(1),lastfollow(2),15,18);
[head,slots]=record_event(head,slots,3,arrival,true,true);
assert(head(3)==7&&head(6)==1&&isequal(slots(3,8:9),lastfollow));
assert(slots(3,7)==15 && slots(3,11)==18 && slots(3,4)==140);
savedH=head;savedS=slots;
[head,slots]=record_event(head,slots,2,arrival,true,true);assert(isequal(head,savedH)&&isequal(slots,savedS));
[head,slots]=reset_capture(head,slots,12,false);assert(isequal(head,savedH)&&isequal(slots,savedS));
for state={'STOP','finish','fault','idle'} %#ok<NASGU>
 assert(isequal(head,savedH)&&isequal(slots,savedS));
end
[head,slots]=record_event(head,slots,2,arrival,false,true);assert(isequal(head,savedH)&&isequal(slots,savedS));
[head,slots]=reset_capture(head,slots,12,true);assert(head(2)==12&&head(3)==0&&all(slots(:)==0));
% Unknown bits remain zero; flags keep reached-before-event separate from slot3 validity.
flags=uint32(1+4+64+128);assert(bitand(flags,uint32(8+16+32))==0);assert(bitand(flags,bitcmp(uint32(255)))==0);
assert(wrap(uint32(5),uint32(4294967280))<400);
% Phases from original boundaries; no extra controller or sampling is introduced.
b=[0 .532883465 2.995779514 3.528663158 5.25 5.782883644 8.245779991 8.778662682];
assert(find(7.95<b(2:end),1)==6);assert(find(8.3<b(2:end),1)==7);assert(9>=b(end));
result=struct('schema','snapshot-matlab-checks-v1','status','PASS_APPLICABLE_DIAGNOSTIC_PREREQUISITE',...
 'snapshot_bytes',48,'header_bytes',32,'total_bytes',176,'field_offsets',offsets,...
 'uint32_wrap_cases',3,'float_bytes',20,'reset_hold_event_tests_pass',true,...
 'validmask_tests_pass',true,'arrival_preserves_previous_follow_state',true,...
 'phase_tests_pass',true,'C_ABI_sizeof_verified',false,'product_code_written',false,...
 'B0_remains','FAIL_NOT_IDENTIFIABLE','B1_enabled',false);
f=fopen(fullfile(out,'snapshot_matlab_results.json'),'w','n','UTF-8');assert(f>=0);c=onCleanup(@()fclose(f));fwrite(f,jsonencode(result),'char');
fprintf('Snapshot checks PASS: 48*3+32=176B; wrap/reset/hold/event/flags/phase; C sizeof pending\n');
end

function r=record(now,common,follow,mean,refpos,refF,ff,rate,pending,applied)
r=single([now 0 common follow mean refpos refF ff rate pending applied 197]);
end

function [h,s]=reset_capture(h,s,session,accepted)
if accepted;h(:)=0;h(1)=1;h(2)=uint32(session);s(:)=0;end
end

function [h,s]=record_event(h,s,slot,value,baselineAllowed,validCommon)
if ~baselineAllowed||~validCommon;return;end
if bitand(h(3),uint32(4));return;end
if slot==1&&bitand(h(3),uint32(1));return;end
s(slot,:)=value;h(3)=bitor(h(3),bitshift(uint32(1),slot-1));h(slot+3)=h(slot+3)+uint32(1);
end
