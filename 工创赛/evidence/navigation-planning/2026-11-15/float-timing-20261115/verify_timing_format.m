% Data-format validation only; no chip timing or dynamics simulation.
E=fileparts(mfilename('fullpath'));
delta=@(start,finish) uint32(mod(double(finish)-double(start),2^32));
assert(delta(uint32(4294967280),uint32(16))==32);
assert(delta(uint32(0),uint32(0))==0);
assert(delta(uint32(0),uint32(4294967295))==uint32(4294967295));
s=struct('count',uint32(0),'min_cycles',uint32(0),'last_cycles',uint32(0),...
 'max_cycles',uint32(0),'sum_cycles',uint64(0),'first_ms',uint32(0),'last_ms',uint32(0));
empty=s;
s=record(s,false,uint32(11),uint32(22),uint32(100));
assert(isequal(s,empty)); assert(s.count==0); % valid=0, not a zero-cost sample
s=record(s,true,uint32(4294967280),uint32(16),uint32(4294967280));
s=record(s,true,uint32(100),uint32(100),uint32(5));
s=record(s,true,uint32(0),uint32(4294967295),uint32(16));
assert(s.count==3 && s.min_cycles==0 && s.max_cycles==uint32(4294967295));
assert(s.sum_cycles==uint64(4294967295)+uint64(32));
assert(delta(s.first_ms,s.last_ms)==32);
snapshot=s; s=record(s,true,uint32(1),uint32(3),uint32(20));
assert(snapshot.count==3 && s.count==4); % value snapshot does not alias writer
hi=uint32(bitshift(snapshot.sum_cycles,-32)); lo=uint32(bitand(snapshot.sum_cycles,uint64(4294967295)));
assert(bitor(bitshift(uint64(hi),32),uint64(lo))==snapshot.sum_cycles);
assert(abs(double(snapshot.sum_cycles)/double(snapshot.count)-1431655775.6666667)<1e-6);
frequencies=[84000000 168000000 180000000];
us=double(snapshot.sum_cycles)./double(snapshot.count)*1e6./frequencies;
assert(all(isfinite(us)) && all(us>0) && abs(us(1)-2*us(2))<1e-8);
% At saturated count, freeze the whole record: max possible sum stays in uint64.
s=empty; s.count=uint32(4294967295); s.sum_cycles=uint64(4294967295)*uint64(4294967295);
frozen=s; s=record(s,true,uint32(0),uint32(4294967295),uint32(1)); assert(isequal(s,frozen));
result=struct('all_passed',true,'version',version,'chip_cycles_measured',false,...
 'kind','uint32 wrap / disabled / valid-zero / bounded sum64 / mean / clock conversion / snapshot format',...
 'frequencies_hz',frequencies,'mean_us',us,'no_zero_clock_conversion',true);
f=fopen(fullfile(E,'matlab_summary.json'),'w'); assert(f>=0); fprintf(f,'%s',jsonencode(result)); fclose(f);
disp('TIMING_FORMAT_PASS');

function s=record(s,enabled,start,finish,ms)
if ~enabled || s.count==uint32(4294967295), return; end
cycles=uint32(mod(double(finish)-double(start),2^32));
if s.count==0, s.first_ms=ms; s.min_cycles=cycles; end
s.count=s.count+uint32(1); s.last_cycles=cycles;
s.min_cycles=min(s.min_cycles,cycles); s.max_cycles=max(s.max_cycles,cycles);
s.sum_cycles=s.sum_cycles+uint64(cycles); s.last_ms=ms;
end
