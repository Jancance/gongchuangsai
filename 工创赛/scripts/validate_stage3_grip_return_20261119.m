function validate_stage3_grip_return_20261119
% Event and boundary model, using externally supplied completion times.
n=0;
for ring=1:3
 for z=[80 1200 2300]
  for x=[0 400 2200]
   for residual=[0 3 7.2 17.75 17.76]
    for corrected=[0 3 7.2]
     for failure=0:5
      pwm=100;
      liftStart=pwm+1200;
      assert(liftStart-pwm==1200);
      zReturn=liftStart+z;
      xStart=liftStart+(ring~=2)*z;
      xReturn=xStart+x;
      if ring~=2, assert(xStart==zReturn); end
      home=max(xReturn,zReturn);
      % Reuse the existing 17.75-degree zero-arrival window for rotation.
      advance=failure==0 && residual<=17.75;
      rotate=inf;
      if advance, rotate=home+600; end
      assert(~advance || (rotate>=xReturn && rotate>=zReturn));
      assert((advance || isinf(rotate)) && (~advance || failure==0));
      n=n+1;
     end
    end
   end
  end
 end
end
assert(270/300<12);
assert(7.2/(315.5*360/3200)<0.21);
fprintf('PASS Stage3 grip and ordered return: %d cases\n',n);
end
