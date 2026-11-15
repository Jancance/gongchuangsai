% Scheduling assertion only; one calculation per new sample, no catch-up loop.
out=fileparts(mfilename('fullpath')); last=uint32(0); calls=[];
for tick=[0:5:100,200,200,205,210,215,220]
 if mod(double(uint32(tick))-double(last),2^32)>=20
  calls(end+1)=tick; last=uint32(tick);
 end
end
assert(isequal(calls,[20 40 60 80 100 200 220]));
r=struct('status','PASS_MATLAB_SCHEDULE_ONLY','prediction_period_ms',20,'normal_input_period_ms',5,'call_ticks_ms',calls,'catch_up_calls',0,'hardware_accessed',false);
f=fopen(fullfile(out,'observer_schedule_matlab_summary.json'),'w');assert(f>=0);fwrite(f,jsonencode(r));fclose(f);disp(jsonencode(r));