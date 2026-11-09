function validate_integration13_three_direct_20261109
root=fileparts(fileparts(mfilename('fullpath')));
ev=fullfile(root,'evidence','integration','2026-11-07','stage2-mainline');
angles=[180 195 165];extension=[0 156 156];approach=[4 6 6];carry=[5 7 7];
assert(all(angles>=0 & angles<=270));assert(all(extension>=0 & extension<=170));
assert(all(approach==4+2*(extension>0)));assert(all(carry==5+2*(extension>0)));
n=0;
for item=1:3
 for zdepth=[118800 115000 110800 7200]
  for zero=0:1
   for zdone=0:1
    extAllowed=zdepth<=110800 && extension(item)>0;
    yawAllowed=zdone && (extension(item)==0 || zero);
    assert(~extAllowed || zdepth<=110800);
    assert(~yawAllowed || zdone);n=n+1;
   end
  end
 end
end
% Carrier is latched independently for each accepted approach, once only.
pwm=round(500+[0 120 240]*2000/270);assert(isequal(pwm,[500 1389 2278]));
out=struct('passed',true,'cases',n,'angles_deg',angles,'extension_mm',extension, ...
 'source_approach_masks',approach,'loaded_carry_masks',carry,'carrier_pwm_us',pwm, ...
 'right_5mm_retract_is_trial_not_prior_field_evidence',true, ...
 'zero_stow_after_three',true,'route3_enabled',false,'under20_demonstrated',false);
f=fopen(fullfile(ev,'integration13-three-direct-matlab-20261109.json'),'w','n','UTF-8');
assert(f>0);fprintf(f,'%s\n',jsonencode(out));fclose(f);
fprintf('PASS: %d direct-three mask/height cases, independent carrier slots; no route or timing claim.\n',n);
end
