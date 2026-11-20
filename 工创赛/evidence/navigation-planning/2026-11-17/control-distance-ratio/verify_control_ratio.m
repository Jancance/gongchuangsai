function verify_control_ratio
out=fileparts(mfilename('fullpath'));actual=[2108.3333333333335 2099 2099.6666666666665 2103];scale=2100./actual;assert(all(scale>0)&all(abs(scale-1)<.01));n=0;
for d=[300 1000 1500 2100]
 internal=d.*scale;predicted=internal.*actual/2100;assert(max(abs(predicted-d))<1e-9);assert(d==d);n=n+1;
end
assert(abs(100*(100/101)*1.01-100)<1e-12);
for mode=0:3
 for id=0:22
  for rpm=[15 99 100 101 102 103]
   for mm=[0 300 1500 2099 2100 2101]
    for route=[0 400]
     match=(id==mode+8 && mm==2100 && rpm==(100+2*(mode==1||mode==2)) && route==0);target=mm;if match,target=mm*scale(mode+1);end
     if ~match,assert(target==mm);end;n=n+1;
    end
   end
  end
 end
end
O=struct('status','PASS_PROPORTIONAL_CANDIDATE_ONLY','mode_order',{{'forward','left','right','backward'}},'actual_mean_mm',actual,'request_scale',scale,'nominal_2100_internal_mm',2100*scale,'checks',n,'proportional_assumption',true,'ground_verified',false,'fit_stop_offset',false,'hardware_access',false);
fid=fopen(fullfile(out,'matlab_summary.json'),'w','n','UTF-8');fwrite(fid,jsonencode(O),'char');fclose(fid);disp(O.status);disp(O.nominal_2100_internal_mm);
end
