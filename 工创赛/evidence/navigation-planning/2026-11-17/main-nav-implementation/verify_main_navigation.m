function verify_main_navigation
out=fileparts(mfilename('fullpath'));A=fileparts(out);C=jsondecode(fileread(fullfile(A,'right1-three-direction-analysis','calibration_candidates.json')));L=jsondecode(fileread(fullfile(out,'left_inputs.json')));tr=L(strcmp({L.group},'train'));d=[tr.model_main]';truth=[tr.ground_main]';k=mean((truth-d)./d);G=C.groups;
O=struct('left_k_main',k,'fixed_weight',.5,'groups',G,'hardware_access',false,'product_changed',false);
h=L(strcmp({L.group},'heldout'));a=[h.model_main]';b=[h.ground_main]';O.left_heldout=struct('baseline_errors_mm',(a-b)','half_errors_mm',(a+.5*k*a-b)','baseline_MAE_mm',mean(abs(a-b)),'half_MAE_mm',mean(abs(a+.5*k*a-b)));
coeff=[k G(1).k_main G(2).k_main G(3).k_main];assert(all(coeff<0)&all(1+.5*coeff>0));n=0;
for phi=[0 90 -90 179 -179]
 T=[cosd(phi) -sind(phi);sind(phi) cosd(phi)];for scope=1:4
  for net=[-100 0 1 1000 2100]
   c=.5*coeff(scope)*max(net,0);if scope==1,v=[0;c];elseif scope==2,v=[0;-c];elseif scope==3,v=[c;0];else,v=[-c;0];end
   assert(norm(T'*(T*v)-v)<1e-9);if net<=0,assert(norm(v)==0);end;n=n+1;
  end
 end
end
O.coordinate_cases=n;O.status='PASS_OFFLINE_MAIN_NAV_CANDIDATES';fid=fopen(fullfile(out,'matlab_summary.json'),'w','n','UTF-8');fwrite(fid,jsonencode(O),'char');fclose(fid);disp(O.status);disp(coeff);disp(O.left_heldout);
end
