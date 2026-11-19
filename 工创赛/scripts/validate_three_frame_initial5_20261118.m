% User-authorized initial180/65 lower bound -5; all later starts retain -2.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-18','three-frame-initial5');
if ~exist(out,'dir'),mkdir(out);end
checks=0;
positions=[-Inf,NaN,-30,-5.01,-5,-4,-3.1146,-3.1036834716796875,-2,-1,0,180,272,272.01,Inf];
for state=0:9
 for prep=0:5
  for axes=[1,2,3,4,6,7]
   for goal=[165,180,195]
    for extension=[0,65,164]
     scope=state==1 && prep==4 && axes==3 && goal==180 && extension==65;
     for work=positions
      old=work>=-2 && work<=272;
      candidate=work>=(-2-3*double(scope)) && work<=272;
      expected=old || (scope && work>=-5 && work<-2);
      assert(candidate==expected);checks=checks+1;
     end
    end
   end
  end
 end
end
for top=[false,true]
 for zero=[false,true]
  for sync=[false,true]
   for parallel=[false,true]
    for go=[false,true]
     scoped=top && zero && sync && parallel && ~go;
     assert(((-3.1146)>=(-2-3*double(scoped)))==scoped);checks=checks+1;
    end
   end
  end
 end
end
% State model for cold and fast prepare: no DM target or X movement before
% cache/broadcast, one broadcast releases both, cancellation keeps go false.
for fast=[false,true]
 for failure=0:3
  work=-3.1146; token=true;go=false;broadcasts=0;dm_target=work;x_moving=false;
  assert(work>=-5);checks=checks+1;
  if failure~=1 % preparation failure
   assert(dm_target==work && ~x_moving && ~go);checks=checks+1;
   if failure==0
    broadcasts=1;x_moving=true;go=true;token=false;dm_target=180;
   end
  end
  if failure~=0,token=false;end
  assert(broadcasts==double(failure==0));assert(go==(failure==0));
  assert(~token);assert(x_moving==(failure==0));checks=checks+4;
 end
end
result=struct('passed',true,'checks',checks,'initial_lower_bound_deg',-5,...
 'later_lower_bound_deg',-2,'hardware_access',false,...
 'scope','only source state1 prep4 DM|EXT exact180/65 top and zero extension, sync parallel before go',...
 'evidence','Actual cb RAM s_start about -3.1146 degrees; extension-only build has no request6 startup proof',...
 'note','Control model only; no physical clearance, drive motion or cross-bus physical skew verification');
fid=fopen(fullfile(out,'matlab-result.json'),'w');fprintf(fid,'%s\n',jsonencode(result));fclose(fid);
fprintf('PASS MATLAB initial -5 %d checks\n',checks);
