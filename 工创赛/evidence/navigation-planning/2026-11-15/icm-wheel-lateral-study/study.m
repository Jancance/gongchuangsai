% Saved-data research only. No firmware/hardware access or endpoint fitting.
out = fileparts(mfilename('fullpath'));
d = jsondecode(fileread(fullfile(out,'inputs.json')));
selfcheck();
g = d.g_mm_s2; scale = g/d.accel_lsb_g;
fig = figure('Visible','off','Position',[50 50 1250 920]);
summaries = cell(numel(d.runs),1); allseries = cell(3,1);
for n = 1:numel(d.runs)
    r = d.runs(n); ir = r.ir;
    ti = [ir.sample_tick_ms]';
    raw = [[ir.accel_x_raw]' [ir.accel_y_raw]' [ir.accel_z_raw]'];
    gyro_xyz = [[ir.gyro_x_raw]' [ir.gyro_y_raw]' [ir.gyro_z_raw]']/d.gyro_lsb_dps;
    gyro = gyro_xyz(:,3);
    pre = ti>=r.t0_ms-2200 & ti<=r.t0_ms-200;
    post = ti>=r.te_ms+2000 & ti<=r.te_ms+8000;
    assert(sum(pre)>=8 && sum(post)>=20);
    b = mean(raw(pre,:),1); bg = mean(gyro(pre));
    ts = [r.t0_ms; ti(ti>r.t0_ms & ti<r.te_ms); r.te_ms];
    t = (ts-r.t0_ms)/1000; T = t(end);
    f = interp1(ti,(raw-b)*scale,ts);
    yaw = unwrap(r.heading(:,2)*pi/180);
    theta = interp1(r.heading(:,1),yaw,ts)-r.reference_left_deg*pi/180;
    omega = interp1(r.rate(:,1),r.rate(:,2)*pi/180,ts);
    alpha = gradient(omega,t); % Offline central derivative; not a causal observer.
    % Sensor(right,back,down) -> body(forward,left,up).
    ab = [-f(:,2), -f(:,1)];
    % a_sensor = a_center + alpha cross r + omega cross(omega cross r).
    rx = -20; ry = 90; % mm; installation record, not a newly calibrated value.
    ac = [ab(:,1)+alpha*ry+omega.^2*rx, ab(:,2)-alpha*rx+omega.^2*ry];
    right = -(sin(theta).*ac(:,1)+cos(theta).*ac(:,2));
    right_no_lever = -(sin(theta).*ab(:,1)+cos(theta).*ab(:,2));
    % Separate exploratory model: propagate gravity with all three gyro axes.
    % Initial gravity direction comes from pre mean; transverse accel bias is unknown.
    body_mean = [-b(2),-b(1),-b(3)]*scale;
    g0 = body_mean/norm(body_mean)*g;
    assumed_accel_bias = body_mean-g0;
    gs = interp1(ti,gyro_xyz-mean(gyro_xyz(pre,:),1),ts)*pi/180;
    gb = [-gs(:,2),-gs(:,1),-gs(:,3)];
    grav = gravity_transport(t,gb,g0);
    body_raw = interp1(ti,[-raw(:,2),-raw(:,1),-raw(:,3)]*scale,ts);
    ag = body_raw-grav-assumed_accel_bias;
    ag(:,1)=ag(:,1)+alpha*ry+omega.^2*rx;
    ag(:,2)=ag(:,2)-alpha*rx+omega.^2*ry;
    right_gyro = -(sin(theta).*ag(:,1)+cos(theta).*ag(:,2));
    forward_gyro = cos(theta).*ag(:,1)-sin(theta).*ag(:,2);
    imu_gyro = double_integral(t,right_gyro);
    imu_gyro_forward = double_integral(t,forward_gyro);
    imu = double_integral(t,right);
    imu_raw = double_integral(t,f(:,1));
    imu_no_lever = double_integral(t,right_no_lever);
    [tw,enc,forward_yaw,lateral_wheel,efwd] = wheel_path(r);
    ey = interp1(tw,enc,ts);
    ve = gradient(ey,t); % Offline centered differences, not an online measurement.
    fused = zeros(numel(t),numel(d.taus_s));
    fused_gyro = fused;
    for k = 1:numel(d.taus_s)
        fused(:,k) = complementary(t,right,ve,d.taus_s(k));
        fused_gyro(:,k) = complementary(t,right_gyro,ve,d.taus_s(k));
    end
    gz = interp1(ti,gyro-bg,ts); hr = omega*180/pi;
    cor = corrcoef(gz,hr);
    gyro_corr = cor(1,2);
    od = r.od; ot = [od.sample_ms]'; oy = 2100-[od.x_mm]';
    use = ot>=tw(1) & ot<=tw(end);
    oe = interp1(tw,enc,ot(use))-oy(use);
    q = struct('run',r.name,'duration_s',T,'pre_samples',sum(pre),...
        'post_samples',sum(post),'bias_pre_raw',b,'bias_pre_std_raw',std(raw(pre,:),0,1),...
        'bias_post_minus_pre_raw',mean(raw(post,:),1)-b,...
        'ruler_right_mm',r.ruler.lateral_right_mm,'ruler_forward_mm',r.ruler.forward_mm,...
        'encoder_right_mm',enc(end),'encoder_forward_mm',efwd(end),...
        'encoder_forward_yaw_right_mm',forward_yaw(end),...
        'encoder_lateral_wheel_right_mm',lateral_wheel(end),...
        'encoder_minus_ruler_right_mm',enc(end)-r.ruler.lateral_right_mm,...
        'encoder_vs_existing_OD_max_abs_mm',max(abs(oe)),...
        'imu_rawX_prebias_only_right_mm',imu_raw(end),...
        'imu_planar_no_lever_right_mm',imu_no_lever(end),...
        'imu_planar_lever_right_mm',imu(end),...
        'imu_final_right_velocity_mm_s',trapz(t,right),...
        'imu_gyro_gravity_right_mm',imu_gyro(end),...
        'imu_gyro_gravity_forward_mm',imu_gyro_forward(end),...
        'gyro_pre_dps',mean(gyro_xyz(pre,:),1),...
        'gyro_post_minus_pre_dps',mean(gyro_xyz(post,:),1)-mean(gyro_xyz(pre,:),1),...
        'transported_gravity_final_body_mm_s2',grav(end,:),...
        'gyroZ_vs_HWT_rate_corr',gyro_corr,...
        'gyroZ_same_sign_RMSE_dps',sqrt(mean((gz-hr).^2)),...
        'gyroZ_opposite_sign_RMSE_dps',sqrt(mean((-gz-hr).^2)),...
        'taus_s',d.taus_s','fused_right_mm',fused(end,:),...
        'fused_minus_ruler_mm',fused(end,:)-r.ruler.lateral_right_mm,...
        'fused_gyro_right_mm',fused_gyro(end,:),...
        'fused_gyro_minus_ruler_mm',fused_gyro(end,:)-r.ruler.lateral_right_mm,...
        'constant_1LSB_position_error_mm',0.5*scale*T^2,...
        'constant_1mg_position_error_mm',0.5*g*0.001*T^2,...
        'constant_0p1deg_gravity_leak_mm',0.5*g*sin(0.1*pi/180)*T^2,...
        'residual_accel_for_1mm_ug',2/(T^2*g)*1e6);
    summaries{n} = q;
    allseries{n} = [t, f(:,1), right,ey,imu_raw,imu,fused,right_gyro,imu_gyro,fused_gyro];
    writematrix(allseries{n},fullfile(out,[r.name '_series.csv']));
    subplot(3,2,2*n-1);
    plot(t,f(:,1),t,right); grid on;
    xlabel('Time from profile start (s)'); ylabel('Right accel proxy (mm/s^2)');
    title([r.name ': sparse 5 Hz samples, planar assumption']);
    legend('raw X minus pre mean','yaw + lever corrected','Location','best');
    subplot(3,2,2*n);
    plot(tw/1000-r.t0_ms/1000,enc,'k','LineWidth',1.4); hold on;
    plot(t,imu,'Color',[0.7 0.7 0.7]);
    plot(t,imu_gyro,'r',t,fused_gyro(:,2),'b');
    plot(T,r.ruler.lateral_right_mm,'go','MarkerFaceColor','g');
    grid on; xlabel('Time from profile start (s)'); ylabel('Right displacement (mm)');
    title([r.name ': endpoint ruler only, no ground trajectory']);
    legend('wheel + HWT','planar IMU','gyro-gravity IMU','offline fusion tau=0.3 s','ruler','Location','best');
end
print(fig,fullfile(out,'comparison.png'),'-dpng','-r125'); close(fig);
result = struct('matlab_version',version,'selfcheck','PASS',...
    'sensor_to_body_matrix',[0 -1 0;-1 0 0;0 0 -1],...
    'integration_assumption','planar motion; pre-run constant mean removes static gravity projection+bias together',...
    'alignment','offline interpolation and centered derivatives use future samples; not online validation',...
    'gyro_gravity_assumption','pre mean initializes gravity direction; unknown transverse accel bias and lever height; sparse gyro integration only',...
    'candidate_selected',false,'control_changed',false);
result.runs = summaries;
write_json(fullfile(out,'matlab_summary.json'),result);
save(fullfile(out,'study_results.mat'),'summaries','allseries');
disp(jsonencode(result));

function y = double_integral(t,a)
y = cumtrapz(t,cumtrapz(t,a));
end

function [ts,y,fy,ly,forward] = wheel_path(r)
ws = {r.wheel1,r.wheel2,r.wheel3,r.wheel4};
ts = [r.t0_ms; r.te_ms];
for j=1:4
    v=ws{j}; assert(v(1,1)<=r.t0_ms && v(end,1)>=r.te_ms);
    ts=[ts;v(v(:,1)>r.t0_ms & v(:,1)<r.te_ms,1)]; %#ok<AGROW>
end
ts=unique(ts); w=zeros(numel(ts),4);
for j=1:4
    v=ws{j}; w(:,j)=interp1(v(:,1),v(:,2),ts)*((-1)^mod(j+1,2))*pi*77/65536;
end
dw=diff(w); dx=sum(dw,2)/4*1.02182540;
dy=(dw(:,1)-dw(:,2)-dw(:,3)+dw(:,4))/4;
psi=interp1(r.heading(:,1),unwrap(r.heading(:,2)*pi/180),ts)-r.reference_left_deg*pi/180;
half=diff(psi)/2; sn=ones(size(half));i=abs(half)>1e-9;sn(i)=sin(half(i))./half(i);
mid=psi(1:end-1)+half;
fy=[0;cumsum(-sn.*dx.*sin(mid))];ly=[0;cumsum(-sn.*dy.*cos(mid))];y=fy+ly;
forward=[0;cumsum(sn.*(dx.*cos(mid)-dy.*sin(mid)))];
end

function p = complementary(t,a,venc,tau)
v=0;p=zeros(size(t));
for j=2:numel(t)
    dt=t(j)-t(j-1);vp=v+0.5*(a(j-1)+a(j))*dt;
    gain=exp(-dt/tau);vn=gain*vp+(1-gain)*venc(j);
    p(j)=p(j-1)+0.5*(v+vn)*dt;v=vn;
end
end

function grav = gravity_transport(t,omega,g0)
grav=zeros(numel(t),3);grav(1,:)=g0;
for j=2:numel(t)
    v=-0.5*(omega(j-1,:)+omega(j,:))*(t(j)-t(j-1));a=norm(v);
    if a<1e-14
        grav(j,:)=grav(j-1,:);
    else
        k=v/a;old=grav(j-1,:);
        grav(j,:)=old*cos(a)+cross(k,old)*sin(a)+k*dot(k,old)*(1-cos(a));
    end
end
end

function selfcheck()
t=(0:0.2:8)';y=double_integral(t,ones(size(t))*2);
assert(max(abs(y-t.^2))<1e-10);assert(all(double_integral(t,zeros(size(t)))==0));
assert(all(complementary(t,zeros(size(t)),zeros(size(t)),0.3)==0));
M=[0 -1 0;-1 0 0;0 0 -1];assert(det(M)==1 && norm(M*M'-eye(3))==0);
assert(isequal(M*[1;0;0],[0;-1;0])); % +sensor X is body right.
rx=-20;ry=90;alpha=0.2;omega=0.3;
sensor=[-alpha*ry-omega^2*rx,alpha*rx-omega^2*ry];
center=[sensor(1)+alpha*ry+omega^2*rx,sensor(2)-alpha*rx+omega^2*ry];
assert(norm(center)<1e-12);
gt=gravity_transport(t,repmat([0.1,0,0],numel(t),1),[0,0,9806.65]);
assert(max(abs(gt(:,2)-9806.65*sin(0.1*t)))<1e-9);
assert(max(abs(sqrt(sum(gt.^2,2))-9806.65))<1e-9);
% Constant-radius forward arc: right position is negative for left yaw.
psi=pi/6;D=1000;arc=-D*sin(psi/2)/(psi/2)*sin(psi/2);
assert(abs(arc-(-D/psi*(1-cos(psi))))<1e-10);
end

function write_json(path,value)
f=fopen(path,'w','n','UTF-8');assert(f>=0);c=onCleanup(@()fclose(f));
fwrite(f,jsonencode(value),'char');
end
