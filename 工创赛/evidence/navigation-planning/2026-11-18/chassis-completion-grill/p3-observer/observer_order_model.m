function observer_order_model
% Discrete-event model of observer failures and wire transaction admission.
checks = 0;
for activeStage = 1:7
    for recovered = [false,true]
        old = link_event(initial(activeStage),false,false);
        fixed = link_event(initial(activeStage),false,true);
        old = link_event(old,recovered,false);
        fixed = link_event(fixed,recovered,true);
        check(fixed.failed && ~fixed.map && fixed.session == 1 + recovered);
        check(~old.failed && ~old.map);
        if recovered && activeStage <= 6
            check(finish_stow(old) && ~finish_stow(fixed));
        end
    end
end
s = initial(8);
s.map = true;
s = link_event(s,false,true);
s = link_event(s,true,true);
check(~s.failed && ~s.map && s.session == 2);

for resolution = {'ACK','TIMEOUT','LINK_LOST'}
    s = initial(8);
    s.cancel = true;
    check(~can_configure(s,true));
    % Transfer mailbox ownership atomically before dispatching STOP.
    s.cancel = false;
    s.owner = true;
    check(~can_configure(s,true));
    s.stop = true;
    s.owner = false;
    check(can_configure(s,false) && ~can_configure(s,true));
    s.stop = false;
    check(can_configure(s,true));
end

for frames = perms(1:3)'
    for bits = 0:7
        before = logical(bitget(bits,1:3));
        old = false(1,3);
        fixed = false(1,3);
        for index = frames'
            if before(index)
                old(index) = true;
                fixed(index) = accepts_reply(20,NaN);
            end
        end
        check(~any(fixed));
        for index = frames'
            if ~before(index)
                old(index) = true;
                fixed(index) = accepts_reply(20,20);
            end
        end
        check(all(old) && (all(fixed) == ~any(before)));
        for index = frames'
            fixed(index) = accepts_reply(20,20);
        end
        check(all(fixed));
    end
end

% STOP preempts queued data and preserves the buffer owned by the DMA.
for launched = [false,true]
    for tail = 0:3
        next = mod(tail+1,4);
        head = mod(tail+3,4);
        check(head ~= tail);
        head = mod(tail+launched,4);
        check((head == tail) == ~launched);
        check(~accepts_reply(22,21) && ~accepts_reply(22,NaN));
        if launched, tail = next; end
        check(tail == head);
        check(accepts_reply(22,22));
    end
end
check(accepts_reply(0,0) && ~accepts_reply(0,65535));

% Prelaunch observations can still be reported as unsolicited data.
parts = [true,false];
check(~all(parts));
parts = false(1,2); % start of the matching scan resets the pairing window
parts(2) = true;
check(~all(parts));
parts(1) = true;
check(all(parts));
fprintf('observer_order_model checks=%u PASS MATLAB %s\n',checks,version('-release'));
    function check(ok)
        assert(ok);
        checks = checks + 1;
    end
end

function s = initial(stage)
s = struct('stage',stage,'ready',true,'session',1,'failed',false, ...
    'map',false,'cancel',false,'owner',false,'stop',false);
end

function s = link_event(s,ready,latch)
if ready && ~s.ready
    s.session = s.session+1;
elseif ~ready
    s.map = false;
    if latch && s.ready && s.stage >= 1 && s.stage <= 7
        s.failed = true;
    end
end
s.ready = ready;
end

function valid = finish_stow(s)
valid = s.ready && ~s.failed;
end

function allowed = can_configure(s,protectStop)
allowed = ~s.cancel && ~s.owner && ~(protectStop && s.stop);
end

function accepted = accepts_reply(requestSeq,launchedSeq)
accepted = requestSeq == launchedSeq;
end
