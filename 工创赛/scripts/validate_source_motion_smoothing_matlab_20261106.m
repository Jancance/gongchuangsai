function validate_source_motion_smoothing_matlab_20261106
% MATLAB first: actual stationary centroids plus declared synthetic motion.
% This is a model, not field motion/latency acceptance.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-06','source-vision-new-pose','motion-smoothing');
if ~exist(out,'dir'),mkdir(out);end
lines=splitlines(string(fileread(fullfile(root,'evidence','integration','2026-11-06','source-vision-new-pose','source-finalstop-M-static-20261106','observations.jsonl'))));
lines=lines(strlength(lines)>0);n=numel(lines);t=zeros(n,1);raw=zeros(n,3,2);
for k=1:n
 r=jsondecode(lines(k));t(k)=r.host_read_completed_monotonic_s;
 for j=1:3,c=[1,2,4];v=r.detection.regions([r.detection.regions.color_id]==c(j));raw(k,j,:)=[v.center_x_px,v.center_y_px];end
end
t=t-t(1);noise=raw-mean(raw,1);
methods={'raw','mean3','median3','mean5','median5','ema027'};
static_results=struct('method',{},'moving_frames',{},'ready_frames',{},'max_speed',{});
for j=1:numel(methods)
 a=trace(raw,t,methods{j},true(n,1));
 static_results(j)=struct('method',methods{j},'moving_frames',sum(a.moving),'ready_frames',sum(a.ready),'max_speed',max(a.speed));
 fprintf('STATIC %s moving%d ready%d max%.3f\n',methods{j},sum(a.moving),sum(a.ready),max(a.speed));
end
scenarios=struct('method',{},'direction',{},'degrees_s',{},'moving_confirmed',{},'premature_ready',{},'stop_delay_s',{},'max_speed',{});
base=squeeze(mean(raw,1));origin=[986.49,323.85];dt=1/30;times=(0:dt:3.5)';start=.5;stop=2.5;
for j=2:numel(methods)
 for sign=[-1,1]
  for speed=[5,10,30,60,120,180]
   angle=sign*deg2rad(speed)*min(max(times-start,0),stop-start);p=zeros(numel(times),3,2);
   for k=1:numel(times)
    rot=[cos(angle(k)),-sin(angle(k));sin(angle(k)),cos(angle(k))];
    p(k,:,:)=((base-origin)*rot'+origin)+squeeze(noise(mod(k-1,n)+1,:,:));
   end
   a=trace(p,times,methods{j},true(numel(times),1));
   moving=times>=start+.3 & times<stop;
   ready=find(a.ready & times>=stop,1);
   if isempty(ready),delay=inf;else,delay=times(ready)-stop;end
   scenarios(end+1)=struct('method',methods{j},'direction',sign,'degrees_s',speed, ...
    'moving_confirmed',any(a.moving(moving)),'premature_ready',sum(a.ready(times<stop)), ...
    'stop_delay_s',delay,'max_speed',max(a.speed)); %#ok<AGROW>
  end
 end
end
for j=2:numel(methods)
 a=scenarios(strcmp({scenarios.method},methods{j}));
 fprintf('MOTION %s no_confirm%d premature%d worst_delay%.3f\n',methods{j},sum(~[a.moving_confirmed]),sum([a.premature_ready]),max([a.stop_delay_s]));
end
report=struct('engine','local MATLAB','release',version('-release'),'static_real_frames',n,'static_results',static_results, ...
 'synthetic_scenarios',scenarios,'synthetic_noise_basis','repeated additive measured stationary image centroid noise; not field dynamic truth', ...
 'assumed_fps',30,'synthetic_speed_degrees_s',[5,10,30,60,120,180],'field_dynamic_accepted',false,'motion_allowed',false);
write_json(fullfile(out,'comparison.json'),report);save(fullfile(out,'comparison.mat'),'report','raw','t','noise');
end
function a=trace(raw,t,method,valid)
n=size(raw,1);filtered=raw;
for k=1:n
 if strcmp(method,'raw'),continue;end
 if strcmp(method,'ema027')
  if k>1,filtered(k,:,:)=.27*raw(k,:,:)+.73*filtered(k-1,:,:);end
 else
  width=str2double(method(end));first=max(1,k-width+1);
  if startsWith(method,'median'),filtered(k,:,:)=median(raw(first:k,:,:),1);else,filtered(k,:,:)=mean(raw(first:k,:,:),1);end
 end
end
speed=zeros(n,1);moving=false(n,1);ready=moving;seen=false;high=0;low=0;
for k=2:n
 interval=t(k)-t(k-1);
 if interval<.02 || interval>.25 || ~valid(k) || ~valid(k-1),high=0;low=0;continue;end
 d=squeeze(filtered(k,:,:)-filtered(k-1,:,:));
 % Preserve existing raw maximum match distance and per-frame identity.
 raw_d=squeeze(raw(k,:,:)-raw(k-1,:,:));
 if any(vecnorm(raw_d,2,2)>80),high=0;low=0;continue;end
 speed(k)=max(vecnorm(d,2,2))/interval;
 if speed(k)>=30
  high=high+1;low=0;if high>=2,seen=true;high=2;moving(k)=true;end
 elseif ~seen
  high=0;
 elseif speed(k)>25
  low=0;
 else
  low=low+1;ready(k)=low>=3;
 end
end
a=struct('speed',speed,'moving',moving,'ready',ready,'filtered',filtered);
end
function write_json(path,value)
fid=fopen(path,'w','n','UTF-8');assert(fid~=-1);fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
