function validate_zero_batch()
% Independent owner model. Peers return ordinary CAN replies without epoch IDs.
checks = 0;
modulus = 2^32;
for base = [1000, modulus-41]
    for release = [0 2 3]
        for finish = [6 7]
            for slot = 0:10:70
                t = mod(base+slot, modulus);
                sent = admit(t,mod(t+release,modulus));
                ack = sent && elapsed(mod(t+finish,modulus),t) <= 6;
                check(sent == (release <= 2));
                check(ack == (release <= 2 && finish <= 6));
                check(~admit(t,mod(t-1,modulus)));
            end
        end
    end
    for stoppedAt = [0 1 2 3 4]
        for badAck = 0:4
            applied = false(1,4);
            confirmed = applied;
            stopping = false;
            for wheel = 1:4
                if stopping, break; end
                applied(wheel) = true;
                % Stop wins a tie with the ACK; mailbox acceptance still counts.
                stopping = wheel == stoppedAt || wheel == badAck;
                confirmed(wheel) = ~stopping;
            end
            check(all(confirmed) == (stoppedAt == 0 && badAck == 0));
            check(all(~confirmed | applied));
            if stopping, check(nnz(applied) == nnz(confirmed)+1); end
        end
    end
    for stopBefore = 1:4
        applied = false(1,4);
        stopping = false;
        for wheel = 1:4
            if wheel == stopBefore, stopping = true; end
            if stopping, break; end
            applied(wheel) = admit(mod(base+20*(wheel-1),modulus),mod(base+20*(wheel-1)+2,modulus));
        end
        check(stopping && nnz(applied) == stopBefore-1 && ~all(applied));
    end

    % Establish the shared map sample while the motion backend is idle.
    time = @(d) mod(base+d,modulus);
    wheel = time([-125 -105 -65 -25]);
    state = time([-305 -225 -145 -65]);
    check(~ready(base,wheel,state,time(-95),time(-100),false,1,1));
    check(ready(base,wheel,state,time(-95),time(-99),false,1,1));
    check(~ready(base,wheel,state,time(-95),time(-99),true,1,1));
    check(~ready(base,wheel,state,time(-95),time(-99),false,1,2));
    check(~ready(base,[wheel(1:3) time(-131)],state,time(-95),time(-99),false,1,1));
    check(~ready(base,[wheel(1:3) time(1)],state,time(-95),time(-99),false,1,1));
    check(~ready(base,wheel,[state(1:3) time(-331)],time(-95),time(-99),false,1,1));

    for finalAck = [499 500 501]
        acks = time([10 30 50 finalAck]);
        samples = time(finalAck+[980 1000 1020 1040]);
        t = time(finalAck+1100);
        % Same millisecond as final ACK is ambiguous without a sequence.
        check(settled(base,acks,samples,time(finalAck+1030),t,zeros(1,4),zeros(1,4)) ...
            == (finalAck < 500));
        check(~settled(base,acks,[samples(1:3) acks(4)],time(finalAck+1030),t,zeros(1,4),zeros(1,4)));
        check(~settled(base,acks,samples,time(finalAck+1030),t,zeros(1,4),[0 0 12 0]));
        check(~settled(base,acks,samples,time(finalAck+1030),t,[0 0 1 0],zeros(1,4)));
        check(~settled(base,acks,[time(finalAck+969) samples(2:4)],time(finalAck+1030),t,zeros(1,4),zeros(1,4)));
        check(~settled(base,acks,time(finalAck+[880 900 920 940]),time(finalAck+950),time(finalAck+999),zeros(1,4),zeros(1,4)));
    end
end
fprintf('ZERO_BATCH_OWNER %d checks PASS MATLAB %s\n',checks,version('-release'));
    function check(ok)
        assert(ok);
        checks = checks + 1;
    end
end

function delta = elapsed(now, before)
delta = mod(now-before,2^32);
end

function yes = admit(slot, at)
yes = elapsed(at,slot) <= 2;
end

function yes = ready(now, wheel, state, heading, rate, backend, epoch, currentEpoch)
yes = ~backend && epoch ~= 0 && epoch == currentEpoch && all(elapsed(now,wheel) <= 130) ...
    && all(elapsed(now,state)<=330) && elapsed(now,heading) < 100 && elapsed(now,rate) < 100;
end

function yes = settled(stop, acks, samples, rate, now, rpm, countChange)
% Test values distinguish stationary zero RPM from unambiguous count motion.
% Encoder jitter tolerance remains an input to the eventual physical model.
lastAck = acks(4);
yes = all(elapsed(acks,stop)<500) && elapsed(now,lastAck)>=1000 ...
    && all(elapsed(samples,lastAck)>0 & elapsed(samples,lastAck)<2^31) ...
    && all(elapsed(now,samples)<=130) && elapsed(rate,lastAck)>0 ...
    && elapsed(now,rate)<100 && all(rpm==0) && all(countChange==0);
end
