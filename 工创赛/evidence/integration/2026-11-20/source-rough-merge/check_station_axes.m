% Mechanical dispatch sequencing precheck. Feedback and geometry are inputs.
% Model checks the safety/receipt contract, not physical clearance or motion.
function check_station_axes
    traces = 0;
    for kind = 1:4
        switch kind
            case 1
                events = repmat([1 2],1,3);
            case 2
                events = [repmat([7 3],1,3),repmat([4 5],1,3)];
            otherwise
                events = repmat([8 6],1,3);
        end
        for calibrated = [false true]
            for elapsed = [0 179999 180001 400000]
                for feedback = [false true]
                    for vision_ok = [false true]
                        ready = (kind < 3 || calibrated) && feedback;
                        [out,stow,done] = replay(events,ready,vision_ok,elapsed);
                        assert(length(out)==(ready*(vision_ok*length(events)+~vision_ok*(kind~=1))+ ...
                            (ready && ~vision_ok && kind==1)*length(events)));
                        assert(done == (ready && (vision_ok || kind==1)));
                        assert(isequal(stow, [1 2 3]*done));
                        traces = traces+1;
                    end
                end
            end
        end
    end
    fprintf('STATION_AXES PASS %d model traces\n',traces);
end

function [out,stow,done] = replay(events,ready,vision_ok,elapsed)
    assert(elapsed>=0);
    out = [];
    stow = [0 0 0];
    done = false;
    if ~ready, return; end
    for action = events
        if any(action == [3 6]) && ~vision_ok, return; end
        out(end+1) = action; %#ok<AGROW>
    end
    stow = [1 2 3]; % Safe lift top -> extension X0 -> DM0.
    done = true;
end
