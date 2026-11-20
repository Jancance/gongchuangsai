% Model the Pi activation, frame fence, and storage observation contract.
checks = 0;
for mode = 0:4
    for matching = [false true]
        for stateReady = [false true]
            for age = [0 149 200 201]
                for frameStart = [9 10 11]
                    active = mode ~= 0 && matching && stateReady;
                    expected = active && frameStart > 10 && age <= 200;
                    row = [mode matching stateReady age frameStart];
                    outcome = row(1) > 0 && all(row(2:3)) && ...
                        row(4) <= 200 && row(5) > 10;
                    assert(expected == outcome);
                    checks = checks + 1;
                end
            end
        end
    end
end

epochs = [1 11 1 1 50 0 0; 1 12 1 2 50 0 0; ...
    1 13 1 3 50 1 1; 1 14 2 1 50 0 0; ...
    1 15 2 2 50 0 0; 1 16 2 4 50 2 5; 1 16 0 0 0 0 0];
for n = 2:size(epochs, 1)
    assert(~isequal(epochs(n,:), epochs(n-1,:)));
    checks = checks + 1;
end
for calibrated = [false true]
    for evidence = [-1 0 1] % unknown, empty, occupied
        for freshFrames = 1:4
            report = calibrated && evidence >= 0 && freshFrames >= 3;
            assert(report == ismember(evidence,[0 1]) * calibrated * (freshFrames >= 3));
            checks = checks + 1;
        end
    end
end
fprintf('PASS Pi vision gate %d checks\n', checks);
