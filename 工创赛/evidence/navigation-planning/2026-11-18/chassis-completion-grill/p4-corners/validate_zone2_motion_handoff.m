function validate_zone2_motion_handoff(output)
% Zone 2 application joins the wheel owner and the eight-leg scheduler.
H=80; checks=0;
for base=[10000 2^32-134]
    for arrival=[0 1]
        step=0; want=[0 2;2 14;14 10;10 22;22 14;14 10;10 22;22 0];
        requests=0; reused=0; stationStops=0;
        for leg=1:8
            run=7; session=leg+29; epoch=4;
            if leg==6 || leg==7
                cached=leg-3;
                assert(isequal(want(leg,:),want(cached,:)));
                reused=reused+1;
            else
                requests=requests+1;
            end
            scheduleActive=true;
            due=wrap(base+step*H);
            ack=wrap(due+66);
            assert(~start(due,ack,15,ack));
            assert(~start(due,ack,7,wrap(ack+1)));
            assert(start(due,ack,15,wrap(ack+1)));
            step=step+1;
            correction=0;
            next=wrap(base+step*H);
            for mask=0:15
                for stale=0:1
                    receipt=[run session epoch 2 next];
                    wrong=receipt; wrong(5)=wrap(wrong(5)+H*stale);
                    apply=commit(receipt,wrong,mask,wrap(next+66));
                    assert(apply==(mask==15 && stale==0));
                    checks=checks+1;
                end
            end
            for field=1:4
                wrong=receipt; wrong(field)=wrong(field)+1;
                assert(~commit(receipt,wrong,15,wrap(next+66)));
                checks=checks+1;
            end
            assert(~commit(receipt,receipt,15,wrap(next+67)));
            if commit(receipt,receipt,15,wrap(next+66)), correction=0.8; end
            assert(correction==0.8);
            step=step+1;
            stopAck=wrap(base+step*H+40);
            t=wrap(stopAck+1100);
            if arrival || leg>1
                assert(~canArrive(scheduleActive,false,15,true,t,stopAck,true));
                assert(~canArrive(scheduleActive,true,7,true,t,stopAck,true));
                assert(~canArrive(scheduleActive,true,15,false,t,stopAck,true));
                assert(~canArrive(scheduleActive,true,15,true,stopAck,stopAck,true));
                assert(~canArrive(scheduleActive,true,15,true,t,stopAck,false));
                assert(canArrive(scheduleActive,true,15,true,t,stopAck,true));
                stationStops=stationStops+1;
            else
                assert(~canArrive(scheduleActive,false,15,true,t,stopAck,true));
                break;
            end
            step=step+15;
            checks=checks+12;
        end
        if arrival, assert(stationStops==8 && requests==6 && reused==2);
        else, assert(stationStops==0 && requests==1 && reused==0); end
        checks=checks+1;
    end
end
fid=fopen(output,'w'); assert(fid>=0);
center=[240 1200]; next=[1200 1200];
for offset=[0 .001 .0059 .0099 .0101 .02]
    for yaw=0:45:315
        actual=center+offset*[cosd(yaw) sind(yaw)];
        points=[actual; center; next];
        if norm(actual-center)<.01, points(2,:)=[]; end
        assert(isequal(points(1,:),actual) && isequal(points(end,:),next));
        assert(all(vecnorm(diff(points),2,2)>=.01));
        checks=checks+2;
    end
end
% A small post-zero displacement re-enters the same terminal correction.
% No new session or section clock is opened until the fresh pose qualifies.
for d=[0 .1 2.49 2.51 3.2]
    for heading=[0 .08 .12 .2]
        initial=hypot(d,0)<=2.5 && abs(heading)<=.10;
        assert(initial==(d<=2.5 && heading<=.10));
        if ~initial
            elapsed=0;
            x=d; h=heading;
            while hypot(x,0)>2.5 || abs(h)>.10
                x=x-.08*min(12,1.4*x);
                h=h-.08*min(3,2.2*h);
                elapsed=elapsed+80;
                assert(elapsed<2000);
            end
        end
        checks=checks+1;
    end
end
fprintf(fid,'%s\n',jsonencode(struct('version',version,'checks',checks,'passed',true)));
fclose(fid);
fprintf('ZONE2_MOTION_HANDOFF_PASS %d\n',checks);
end

function ok=start(batch,ack,mask,sample)
ok=mask==15 && elapsed(ack,batch)<=66 && newer(sample,ack);
end

function ok=commit(want,got,mask,lastAck)
ok=isequal(want,got) && mask==15 && elapsed(lastAck,want(5))<=66;
end

function ok=canArrive(active,normalStop,stopMask,quiet,sample,stopAck,poseInStation)
ok=active && normalStop && stopMask==15 && quiet && newer(sample,stopAck) && poseInStation;
end

function ok=newer(stamp,anchor)
delta=elapsed(stamp,anchor);
ok=delta>0 && delta<2^31;
end

function t=elapsed(now,before)
t=wrap(now-before);
end

function t=wrap(now)
t=mod(now,2^32);
end
