function replay_joint_corners(p4file,certificate,output)
% Numerical cross-check with explicit wire quantization, independent wheels,
% encoder observations and HWT histories. Source timestamps in this fixture
% are the device sample instants; real devices do not acknowledge that fact.
d=jsondecode(fileread(p4file)); cert=jsondecode(fileread(certificate));
C=struct([]);
for k=1:8
    a=d.routes(k).corners;
    if ~isempty(a), C=[C;reshape(a(strcmp({a.reason},'ok')),[],1)]; end %#ok<AGROW>
end
s={d.outer_both_directions,d.occupied_detour,d.other_angles,d.adjacent_accepted};
for k=1:numel(s), C=[C;reshape(s{k},[],1)]; end %#ok<AGROW>
L=cert.assumptions; H=L.H; W=[1 1;1 -1;1 -1;1 1]/L.K;
rows=struct([]); total=0; totalStable=0; unalignedExposed=0;
for i=1:numel(C)
    c=C(i); bounds=cert.rows(i); n=bounds.batches; B=bounds.brake_batches*H;
    T=n*H; P=c.control_points_mm; itsHeading=c.heading_deg*pi/180;
    R=[cos(itsHeading) -sin(itsHeading);sin(itsHeading) cos(itsHeading)];
    worstPos=0; worstYaw=0; maxAligned=0; maxBad=0; maxSensor=0; samples=0;
    prefixes=0:n; % Analytic certificate covers every real-valued prefix.
    for prefix=prefixes
        for pattern=0:1
            reaction=L.age+H+L.phase; sw=prefix*H+reaction;
            t0=-H; tEnd=ceil((sw+B+2*H)/H)*H;
            [tt,who]=ndgrid((-2:ceil(tEnd/H))*H,(1:4)); tt=tt+(who-1)*.020;
            s0=tt(:); who=who(:);
            if pattern==0
                late=zeros(size(s0));
            else
                late=L.late*mod(round((s0-(who-1)*.020)/H)+who,2);
            end
            qpos=pos(P,T,sw,B,s0+H)-pos(P,T,sw,B,s0);
            q=round(10*sum((qpos*R).*W(who,:),2)/H)/10;
            dir=xor(q<0,mod(who,2)==0);
            q=abs(q).*(1-2*xor(dir,mod(who,2)==0));
            [ts,order]=sort(s0+late);
            who=who(order); q=q(order);
            held=zeros(numel(ts),4);
            for w=1:4
                ch=find(who==w); idx=interp1(ch,(1:numel(ch))',(1:numel(ts))','previous','extrap');
                idx(1:ch(1))=1; idx(ch(end):end)=numel(ch); held(:,w)=q(ch(idx));
            end
            knots=unique([t0;ts(ts>t0 & ts<tEnd);tEnd]);
            ind=interp1(ts,(1:numel(ts))',knots,'previous'); vel=held(ind,:)*L.K;
            dt=diff(knots); ww=vel(1:end-1,:);
            vx=mean(ww,2); vy=ww*[1;-1;-1;1]/4;
            omega=ww*[-1;1;-1;1]/(4*L.lever); dh=omega.*dt;
            yaw=itsHeading+[0;cumsum(dh)];
            ang=yaw(1:end-1)+dh/2; sf=ones(size(dh)); nz=abs(dh)>1e-10;
            sf(nz)=sin(dh(nz)/2)./(dh(nz)/2);
            xy=pos(P,T,sw,B,t0)+[zeros(1,2);cumsum([cos(ang).*vx-sin(ang).*vy,...
                sin(ang).*vx+cos(ang).*vy].*(dt.*sf),1)];
            counts=[zeros(1,4);cumsum(ww.*dt,1)]*65536/(77*pi);
            truth=pos(P,T,sw,B,knots);
            posError=max(vecnorm(xy-truth,2,2)); angleError=max(abs(yaw-itsHeading));
            assert(all(held(ind(end),:)==0) && norm(xy(end,:)-xy(end-1,:))<1e-8);
            assert(posError<=bounds.position_error_mm+1e-7);
            assert(angleError<=bounds.wheel_distance_error_mm/L.lever+1e-8);
            worstPos=max(worstPos,posError); worstYaw=max(worstYaw,angleError*180/pi);
            if prefix==n
                [nSamples,g,bad,sensor]=observe(knots,xy,counts,yaw,P,T,sw,B);
                samples=samples+nSamples; maxAligned=max(maxAligned,g);
                maxBad=max(maxBad,bad); maxSensor=max(maxSensor,sensor);
                assert(sensor<=L.track && g<=bounds.position_error_mm+L.track);
                unalignedExposed=unalignedExposed+(bad>g+0.5);
            end
            total=total+1;
        end
    end
    totalStable=totalStable+samples;
    rows=[rows;struct('id',bounds.id,'prefix_traces',2*(n+1),'stop_endpoints',2*(n+1),...
        'pose_samples',samples,'integrated_error_mm',worstPos,'induced_yaw_deg',worstYaw,...
        'common_time_error_mm',maxAligned,'wrong_now_error_mm',maxBad,...
        'interpolated_sensor_model_error_mm',maxSensor)]; %#ok<AGROW>
end
% uint32_t ring comparisons (<2^31 separation), history and identity anti-tests.
base=2^32-54; stamps=mod(base+[0 80 160 240 320],2^32);
assert(mod(stamps(5)-stamps(3),2^32)==160 && mod(stamps(3)-stamps(5),2^32)>2^31);
assert(~admissible(stamps(5),stamps(3),80,false,true));
assert(~admissible(stamps(5),stamps(3),101,true,true));
assert(~admissible(stamps(5),stamps(3),80,true,false));
assert(~admissible(stamps(5),mod(stamps(3)-1,2^32),80,true,true));
assert(admissible(stamps(5),stamps(3),80,true,true));
assert(unalignedExposed==2*numel(C));
result=struct('version',version,'rows',{rows},'complete_brake_traces',total,...
  'common_pose_comparisons',totalStable,'naive_timestamp_counterexamples',unalignedExposed);
f=fopen(output,'w'); assert(f>=0); fprintf(f,'%s\n',jsonencode(result)); fclose(f);
fprintf('P4_JOINT_REPLAY traces=%d commonPose=%d wrongComparisons=%d\n',total,totalStable,unalignedExposed);
end

function valid=admissible(now,stamp,increment,sameIdentity,bracketed)
valid=mod(now-stamp,2^32)<=160 && increment>=20 && increment<=100 && sameIdentity && bracketed;
end

function p=pos(P,T,sw,B,t)
z=min(1,max(0,(t-sw)/B)); s=min(t,sw)+B*(z-z.^3+.5*z.^4);
u=min(1,max(0,s/T)); p=zeros(numel(u),2);
for k=0:5
    p=p+(nchoosek(5,k)*u.^k.*(1-u).^(5-k))*P(k+1,:);
end
p=p+min(s/T,0)*(5*(P(2,:)-P(1,:)))+max(s/T-1,0)*(5*(P(end,:)-P(end-1,:)));
end

function [samples,aligned,wrong,sensor]=observe(t,xy,count,yaw,P,T,sw,B)
% Normal electrical response timestamps equal the end of each model read.
% Reproduce oldest-of-latest common time, then interpolate four *positions*.
batch=.08; wheelMm=77*pi/65536;
idx=(0:floor((t(end)-.06)/batch))'; clock=reshape((idx*batch+[.01 .03 .05])',[],1);
address=1+mod((0:numel(clock)-1)',4); read=clock+.006*mod((1:numel(clock))',2);
ht=(0:.010:t(end))'; hSamples=round(interp1(t,yaw,ht)*180/pi*100)/100*pi/180;
rc=cell(1,4); rt=cell(1,4);
for w=1:4
    rt{w}=read(address==w); rc{w}=round(interp1(t,count(:,w),rt{w}));
end
samples=0; aligned=0; wrong=0; sensor=0; prevCommon=-inf; previous=[]; odom=[];
for now=(3:floor((t(end)-.08)/batch))*batch
    latest=cellfun(@(a) a(find(a<=now,1,'last')),rt);
    common=min([latest ht(find(ht<=now,1,'last'))]);
    assert(now-common<=.160+1e-9);
    if common<=prevCommon+1e-10, continue; end
    if isfinite(prevCommon), assert(common-prevCommon<=.100+1e-9 && common-prevCommon>=.020-1e-9); end
    current=zeros(1,4);
    for w=1:4
        assert(any(rt{w}<=common) && any(rt{w}>=common & rt{w}<=now));
        current(w)=interp1(rt{w},rc{w},common)*wheelMm;
    end
    h=interp1(ht,hSamples,common); truePose=interp1(t,xy,common);
    if isempty(odom)
        odom=truePose; % Known fixture origin, disclosed; measures increments from here.
    else
        w=current-previous; dx=mean(w); dy=w*[1;-1;-1;1]/4;
        half=(h-previousYaw)/2; scale=1; if abs(half)>1e-10, scale=sin(half)/half; end
        ang=previousYaw+half;
        odom=odom+scale*[cos(ang)*dx-sin(ang)*dy,sin(ang)*dx+cos(ang)*dy];
    end
    sensor=max(sensor,norm(odom-truePose)); aligned=max(aligned,norm(odom-pos(P,T,sw,B,common)));
    wrong=max(wrong,norm(odom-pos(P,T,sw,B,now))); samples=samples+1;
    previous=current; previousYaw=h; prevCommon=common;
end
end
