% Mandatory pre-implementation MATLAB math checks. No hardware access.
out=fileparts(mfilename('fullpath'));cases=0;max_error=0;
t=(0:0.005:12)';
% Independently known polynomial motion, with fully causal delayed wheel anchors.
for a=[-150 0 150]
 for j=[-20 0 20]
  acceleration=a+j*t;I=cumtrapz(t,acceleration);J=cumtrapz(t,I);
  truth_p=80*t+0.5*a*t.^2+j*t.^3/6;
  truth_v=80+a*t+0.5*j*t.^2;
  for interval=[0.022 0.1 0.2]
   for age=[0 0.05 0.15]
    now=8;one=now-age;zero=one-interval;
    p0=80*zero+0.5*a*zero^2+j*zero^3/6;
    p1=80*one+0.5*a*one^2+j*one^3/6;
    u=(p1-p0)/interval;k=(interp1(t,J,one)-interp1(t,J,zero))/interval;
    pred=p1+u*age+interp1(t,J,now)-interp1(t,J,one)-k*age;
    vel=u+interp1(t,I,now)-k;
    error=abs(pred-interp1(t,truth_p,now));max_error=max(max_error,error);
    assert(error<0.01 && abs(vel-interp1(t,truth_v,now))<0.01);
    cases=cases+1;
   end
  end
 end
end
% Constant accelerometer error has only a short-window effect, not a T^2 run drift.
for bias=[-9.80665 0 9.80665]
 for interval=[0.022 0.1 0.2]
  for age=[0.05 0.15]
   now=8;one=now-age;zero=one-interval;
   I=bias*t;J=0.5*bias*t.^2;k=(interp1(t,J,one)-interp1(t,J,zero))/interval;
   err=interp1(t,J,now)-interp1(t,J,one)-k*age;
   expected=0.5*bias*(interval*age+age^2);
   assert(abs(err-expected)<0.001);cases=cases+1;
  end
 end
end
% Planar lever correction: sensor-only circular acceleration must cancel.
for omega=[-0.1 0 0.1]
 r=[-20;90];theta=omega*t;
 world=[r(1)*cos(theta)-r(2)*sin(theta),r(1)*sin(theta)+r(2)*cos(theta)];
 v=omega*[-world(:,2),world(:,1)];a=-omega^2*world;
 I=cumtrapz(t,a);J=cumtrapz(t,I);Ic=I-v;Jc=J-world;
 % Arbitrary constant/linear integration offsets cancel in the predictor.
 now=8;one=7.85;zero=7.75;
 K=(interp1(t,Jc,one)-interp1(t,Jc,zero))/0.1;
 pp=interp1(t,Jc,now)-interp1(t,Jc,one)-K*0.15;
 vv=interp1(t,Ic,now)-K;
 assert(max(abs(pp))<0.001 && max(abs(vv))<0.001);cases=cases+1;
end
% Same second-order gravity transport intended for the C observer, at 200Hz.
g=[0 0 9806.65];initial=g;omega=[0.03 -0.02 0.01];dt=0.005;
for k=1:2000
 c=cross(omega,g);g=g-c*dt+0.5*cross(omega,c)*dt^2;
 g=g/norm(g)*9806.65;
end
axis=omega/norm(omega);angle=-norm(omega)*10;
exact=initial*cos(angle)+cross(axis,initial)*sin(angle)+axis*dot(axis,initial)*(1-cos(angle));
assert(norm(g-exact)<0.001);cases=cases+1;
% Mount and path signs; no physical coordinate inference is made from peaks.
M=[0 -1 0;-1 0 0;0 0 -1];assert(det(M)==1 && isequal(M*[1;0;0],[0;-1;0]));
assert(abs(-(sin(pi/2)*1+cos(pi/2)*0)+1)<1e-12);cases=cases+2;
% Wire maximum size: tag + 25 signed decimal words + CRC/CRLF fits <384 bytes.
assert(4+25*12+6<384);cases=cases+1;
result=struct('status','PASS_MATLAB_MATH_ONLY','cases',cases,'max_polynomial_position_error_mm',max_error,...
 'rate_Hz',200,'history_samples',128,'new_stopping_conditions',0,...
 'controls_modified',false,'ground_accuracy_validated',false,...
 'limits','synthetic constant/jerk/planar lever/gyro transport; not measured noise, timing, vibration, slip, or board validation');
f=fopen(fullfile(out,'observer_matlab_summary.json'),'w');assert(f>=0);fwrite(f,jsonencode(result));fclose(f);
disp(jsonencode(result));
