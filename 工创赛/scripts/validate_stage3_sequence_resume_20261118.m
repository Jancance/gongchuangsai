% Existing MCU signed uint32 serial ordering across a Pi process restart.
old=uint32(6500);reset=uint32(1);resumed=uint32(6501);
newer=@(a,b)mod(double(a)-double(b),2^32)>0 && mod(double(a)-double(b),2^32)<2^31;
assert(~newer(reset,old));assert(newer(resumed,old));assert(~newer(old,old));
assert(newer(uint32(2),uint32(2^32-2)));
assert(~newer(uint32(2^32-2),uint32(2)));
% Only valid CRC O31 messages with exact run and pose qualify for resumption.
records=[2 16 6450;2 16 6500;2 18 9000;3 16 10000];
same=records(records(:,1)==2 & records(:,2)==16,:);seed=max(same(:,3));
assert(seed==6500 && newer(uint32(seed+1),old));
% New pose resets the MCU sequence as before; larger valid host sequence is OK.
assert(newer(uint32(seed+1),uint32(0)));
root=fileparts(fileparts(mfilename('fullpath')));
path=fullfile(root,'evidence','integration','2026-11-18','stage3-resume','row-sequence-matlab.json');assert(~isfile(path));
result=struct('status','PASS_SEQUENCE_RESUME_MATH','matlab_version',version,'same_run_pose_seed',seed, ...
 'MCU_sequence_reset',false,'stale_packet_rejection_preserved',true,'hardware_tested',false);
fid=fopen(path,'w','n','UTF-8');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(result));clear cleanup;
fprintf('SEQUENCE RESUME MATLAB PASS; reset=1 rejected; resume=6501 accepted after6500\n');
