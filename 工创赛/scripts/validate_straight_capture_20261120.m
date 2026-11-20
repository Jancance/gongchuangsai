function validate_straight_capture_20261120(output)
% Nominal reference from the existing seven-phase distance controller.
K=pi*77/60;
factors=K*[1.02182540 1 1 1.02182540];
rpm=[100 102 102 100];
for distance=[1000 2000]
    for direction=0:3
        factor=factors(direction+1);
        cap=min(400,factor*(rpm(direction+1)-2));
        A=150-4*factor;
        J=300-12*factor;
        tj=min(A/J,sqrt(cap/J));
        ta=max(0,cap/(J*tj)-tj);
        tv=0;
        if distance>=cap*(2*tj+ta)
            tv=(distance-cap*(2*tj+ta))/cap;
        else
            tj=A/J;
            assert(distance>=2*A*tj^2);
            ta=(sqrt(tj^2+4*distance/A)-3*tj)/2;
        end
        dt=[tj ta tj tv tj ta tj];
        position=0; speed=0; accel=0;
        signs=[1 0 -1 0 -1 0 1];
        for phase=1:7
            t=dt(phase); jerk=J*signs(phase);
            position=position+speed*t+accel*t^2/2+jerk*t^3/6;
            speed=speed+accel*t+jerk*t^2/2;
            accel=accel+jerk*t;
        end
        peak=J*tj*(tj+ta);
        assert(abs(position-distance)<.001 && abs(speed)<1e-8 && abs(accel)<1e-8);
        assert(peak<=400+1e-8 && peak/factor+2<=rpm(direction+1)+1e-8);
        assert(sum(dt)+2<12.779);
        assert((distance==1000 && tv==0) || (distance==2000 && tv>1));
        row=struct('direction',direction,'distance_mm',distance,'peak_mm_s',peak,...
            'cruise_s',tv,'time_s',sum(dt));
        index=4*(distance/1000-1)+direction+1;
        rows(index)=row; %#ok<AGROW>
        fprintf('%d mm mode %d: %.3f mm/s, cruise %.3f s, total %.3f s\n',distance,direction,peak,tv,sum(dt));
    end
end
fid=fopen(output,'w');
assert(fid>=0);
fprintf(fid,'%s\n',jsonencode(struct('version',version,'passed',true,'physical_test',false,'rows',rows)));
fclose(fid);
end
