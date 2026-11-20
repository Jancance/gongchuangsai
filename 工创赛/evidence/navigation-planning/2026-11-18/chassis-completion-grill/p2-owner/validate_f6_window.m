function validate_f6_window
checks = 0;
for slot = [100,2^32-3,2^32-1,0]
    for entry = -2:9
        for prep = 0:4
            for fifoClear = [false,true]
                for mailboxAvailable = [false,true]
                    for rx = 0:9
                        for ackMatches = [false,true]
                            launched = entry>=0 && entry<=2 && entry+prep<=2 && fifoClear;
                            accepted = launched && mailboxAvailable;
                            confirmed = accepted && rx>=entry+prep && rx<=6 && ackMatches;
                            actual = transport(slot,stamp(slot,entry),stamp(slot,entry+prep), ...
                                fifoClear,mailboxAvailable,stamp(slot,rx),ackMatches);
                            check(isequal(actual,[launched,accepted,confirmed]));
                        end
                    end
                end
            end
        end
    end
end
fprintf('validate_f6_window checks=%u PASS MATLAB %s\n',checks,version('-release'));
    function check(ok)
        assert(ok);
        checks = checks+1;
    end
end

function evidence = transport(slot,entry,now,fifoClear,hasMailbox,reply,ack)
launched = span(entry,slot)<=2 && span(now,slot)<=2 && fifoClear;
possiblyApplied = launched && hasMailbox;
confirmed = possiblyApplied && span(reply,now)<=span(stamp(slot,6),now) && ack;
evidence = [launched,possiblyApplied,confirmed];
end

function out = stamp(t,dt)
out = mod(t+dt,2^32);
end

function out = span(a,b)
out = mod(a-b,2^32);
end
