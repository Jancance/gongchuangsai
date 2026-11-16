function validate_right1_candidate
% Engineering arithmetic only; no dynamics, hardware, or product writes.
out=fileparts(mfilename('fullpath'));diary(fullfile(out,'matlab_diary.log'));
beta=.01;f=pi*77/60;cases=0;maxWheel=0;maxErrors=zeros(1,3);maxWheelError=0;
for b=0:102
 for requested=unique([-2:.025:2,-1.999,-.051,-.049,0,.049,.051,1.999])
  h=double(max(-min(single(b),single(2)),min(min(single(b),single(2)),single(requested))));t=double(single(beta)*single(b));
  [ok,q]=mix(b,requested);assert(ok);maxWheelError=max(maxWheelError,max(abs(q-ideal)));assert(all(abs(q-ideal)<=.0501));
  F=sum(q)/4;L=(q(1)-q(2)-q(3)+q(4))/4;
  H=(-q(1)+q(2)-q(3)+q(4))/4;
  errors=abs([F-t,L+b,H-h]);maxErrors=max(maxErrors,errors);
  assert(all(errors<=.050001));assert(all(abs(q)<=106));
  assert(all(abs(q)<130));assert(abs(h)<=min(b,2)+1e-10);
  if b==0,assert(all(q==0));end
  maxWheel=max(maxWheel,max(abs(q)));cases=cases+1;
 end
end
for b=[-1 103 NaN Inf -Inf]
 [ok,q]=mix(b,0);assert(~ok && all(q==0));
end
for h=[-2.001 2.001 NaN Inf -Inf]
 [ok,q]=mix(100,h);assert(~ok && all(q==0));
end
quant=zeros(103,5);
for b=0:102
 [ok,q]=mix(b,0);assert(ok);t=double(single(beta)*single(b));
 quant(b+1,:)=[b,t,floor(t+.5),sum(q)/4,(sum(q)/4-t)*f];
end
assert(all(quant(1:50,3)==0));assert(all(abs(quant(1:5,4))<1e-10));
assert(quant(6,4)>=0 && quant(6,4)<=.100001);assert(abs(quant(7,4)-.1)<1e-10 && abs(quant(103,4)-1)<1e-10);
% Scope model: only debug candidate mode2 plans10/12/13 exact tuple.
scopeCases=0;enabledCases=0;
for active=0:1,for flag=0:1,for route=0:1,for plan=7:21
 for mode=0:3,for rpm=[100 102],for mm=[300 1500 2100]
  pair=(plan==10 && mm==2100)||(plan==12 && mm==300)||(plan==13 && mm==1500);
  enabled=active && flag && ~route && mode==2 && rpm==102 && pair;
  if enabled,enabledCases=enabledCases+1;assert(ismember(plan,[10 12 13]));end
  scopeCases=scopeCases+1;
 end,end,end
end,end,end,end
assert(enabledCases==3);
% Continuous reference reserve includes original follow A4RPM/s,J12RPM/s2.
A=150/(1+beta)-f*4;J=300/(1+beta)-f*12;assert(A>0 && J>0);
assert((A+f*4)*(1+beta)<=150+1e-10);
assert((J+f*12)*(1+beta)<=300+1e-10);
profiles=repmat(struct('distance_mm',0,'peak_mm_s',0,'duration_s',0,'phase_s',zeros(1,7)),1,3);
for n=1:3
 distances=[300 1500 2100];distance=distances(n);v=400;tj=A/J;ta=v/A-tj;
 if distance<v*(2*tj+ta)
  v=(-A*A/J+sqrt((A*A/J)^2+4*A*distance))/2;
  if v>=A*A/J,tj=A/J;ta=v/A-tj;
  else,tj=(distance/(2*J))^(1/3);ta=0;v=J*tj*tj;end
 end
 tv=max(0,(distance-v*(2*tj+ta))/v);d=[tj ta tj tv tj ta tj];
 js=J*[1 0 -1 0 -1 0 1];x=[0 0 0];
 for k=1:7
  dt=d(k);x=[x(1)+x(2)*dt+.5*x(3)*dt^2+js(k)*dt^3/6,...
   x(2)+x(3)*dt+.5*js(k)*dt^2,x(3)+js(k)*dt];
 end
 assert(abs(x(1)-distance)<1e-8 && max(abs(x(2:3)))<1e-8);
 assert(sum(d)+2<12.779 && v<=400+1e-9);
 profiles(n)=struct('distance_mm',distance,'peak_mm_s',v,'duration_s',sum(d),'phase_s',d);
end
% Forward/left/yaw orthogonality is independent of t; left path stays unchanged.
for b=0:102,for h=[-2 0 2]
 h=max(-min(b,2),min(min(b,2),h));leftTrim=floor(.02*b+.5);
 left=[b-leftTrim-h,-b-leftTrim+h,-b-leftTrim-h,b-leftTrim+h];
 assert(abs((left(1)-left(2)-left(3)+left(4))/4-b)<1e-10);
end,end
summary=struct('status','PASS_ARITHMETIC_ONLY','matlab_version',version,...
 'beta',beta,'command_cases',cases,'invalid_cases',10,'scope_cases',scopeCases,...
 'enabled_scope_cases',enabledCases,'wheel_peak_rpm',maxWheel,'max_wheel_quantization_error_rpm',maxWheelError,...
 'max_quantization_error_F_L_H_rpm',maxErrors,'reference_A_mm_s2',A,...
 'reference_J_mm_s3',J,'profiles',profiles,'hardware_accessed',false,...
 'product_written',false,'float_model','MATLAB single per C float operation','continuous_wheel_jerk_certified',false,...
 'limits',{{'No dynamics, slip, heading response or ground accuracy prediction',...
 'Direct float trim disappears below5RPM at zero heading after0.1RPM encoding',...
 '0.1RPM wheel quantization changes F,L,H by up to0.05RPM',...
 'Left2percent integer trim and other modes must remain unchanged',...
 'Actual C, compile-time guards and inactive regression need separate verification'}});
writematrix(quant,fullfile(out,'low_speed_quantization.csv'));
fid=fopen(fullfile(out,'matlab_summary.json'),'w');assert(fid>=0);
fprintf(fid,'%s',jsonencode(summary));fclose(fid);disp(summary);diary off;
end
function [ok,q]=mix(b,h)
q=zeros(1,4);ok=false;
if ~(isfinite(b)&&b>=0&&b<=102&&b==floor(b)&&isfinite(h)&&h>=-2&&h<=2),return;end
b=single(b);h=single(h);h=max(-min(b,single(2)),min(min(b,single(2)),h));t=single(.01)*b;
w=[-b+t-h,b+t+h,b+t-h,-b+t+h];
q=double(sign(w)).*floor(double(abs(w)*single(10)+single(.5)))/10;ok=all(abs(q)<=106);
end