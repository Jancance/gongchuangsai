function validate_zone2_initial_zero()
% Bound the wait from the first fully ACKed zero batch to independent quiet.
checks=0;
for start=[1000 2^32-120]
    for ackDelay=0:6
        for wheelDelay=2:6
            for rotation=0:3
                T=2^32;
                elapsed=@(a,b) mod(a-b,T);
                anchor=mod(start+60+ackDelay,T);
                last=zeros(1,4); seen=zeros(1,4); good=false(1,4);
                reads=0; committed=0; queued=1; success=false;
                for slot=0:10:580
                    t=mod(start+slot,T); phase=mod(slot/10,8);
                    if phase==0
                        assert(queued==committed+1);
                    elseif phase==6
                        committed=queued;
                        assert(elapsed(t+ackDelay,anchor)==slot-60);
                    elseif ismember(phase,[1 3 5]) && slot>60
                        wheel=mod(reads+rotation,4)+1;
                        stamp=mod(t+wheelDelay,T);
                        if ~good(wheel), seen(wheel)=stamp; end
                        good(wheel)=true;
                        last(wheel)=stamp;
                        reads=reads+1;
                    end
                    if committed==queued
                        quiet=all(good) && all(elapsed(last,seen)>=1) && ...
                            all(elapsed(t+wheelDelay,last)<=130);
                        if quiet
                            assert(committed>=2 && all(elapsed(last,anchor)>0));
                            assert(slot<=320);
                            success=true;
                            break;
                        end
                        queued=committed+1;
                    end
                end
                assert(success);
                checks=checks+1;
            end
        end
    end
end
fprintf('ZONE2 initial repeated zero %d timing cases PASS MATLAB %s\n',checks,version('-release'));
end
