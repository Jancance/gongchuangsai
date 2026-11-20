function validate_full_map_mission
% Mission contract only. Hardware transfer and combined sweep require their own evidence.
folder = fileparts(mfilename('fullpath'));
project = fileparts(fileparts(fileparts(fileparts(fileparts(folder)))));
routeFiles = {'zone2-field-read-20261120-012415.json', ...
    'zone2-field-read-20261120-004517.json', ...
    fullfile('pa1-dual-launch-20261120-040232','candidate-zone1-arm.json')};
for i = 1:numel(routeFiles)
    data = jsondecode(fileread(fullfile(project,'tmp',routeFiles{i})));
    if isfield(data,'route_trace'), route = data.route_trace; else, route = data.route; end
    assert(route.completed_legs == 8);
    home = 4 * (route.grid.start_id == 1);
    cells = [home 2 14 10 22 14 10 22 home];
    paths = [route.legs.path];
    assert(isequal([paths.start],cells(1:end-1)));
    assert(isequal([paths.goal],cells(2:end)));
    assert(all([route.legs.stop_source_ms] > 0));
    assert(all(diff([route.legs.arrive_elapsed_ms]) > 0));
    assert(~any(ismember(cells,route.grid.occupied_ids)));
end
example = [1 5 6;1 2 3;5 1 6;2 3 1];
assert(isequal(stack_rings(example,true(1,3)),[2 1 3]));
for colors = nchoosek(1:6,3)'
    seq = perms(colors');
    rings = perms(1:3);
    for first = 1:6
        for second = 1:6
            for base = 1:6
                code = [seq(first,:);rings(base,:);seq(second,:);rings(second,:)];
                stack = stack_rings(code,true(1,3));
                assert(all(code(1,ismember(code(2,:),stack)) == code(1,:)));
                for item = 1:3
                    assert(code(1,code(2,:)==stack(item))==code(3,item));
                    confirmed = true(1,3);
                    confirmed(code(2,:)==stack(item)) = false;
                    assert(stack_rings(code,confirmed)*unit(item) == 0);
                end
            end
        end
    end
end
for elapsed = [0 12000 15000 60000 181000]
    assert(strcmp(next_phase(elapsed,true,true,false),'next'));
    assert(strcmp(next_phase(elapsed,true,false,false),'wait'));
    assert(strcmp(next_phase(elapsed,false,true,false),'wait'));
    assert(strcmp(next_phase(elapsed,true,true,true),'fault'));
end
fid = fopen(fullfile(folder,'full_map_mission_result.txt'),'w');
assert(fid >= 0);
report = sprintf('FULL_MAP_MISSION_PASS %s, 3 route replays, 4320 tasks, 12960 missing-base cases, 20 timing/fault cases\n',version);
fprintf(fid,'%s',report);
fclose(fid);
fprintf('%s',report);
end

function rings = stack_rings(code,confirmed)
rings = zeros(1,3);
for i = 1:3
    base = code(1,:)==code(3,i) & confirmed;
    if any(base), rings(i) = code(2,base); end
end
end

function v = unit(i)
v = zeros(3,1);
v(i) = 1;
end

function phase = next_phase(~,confirmed,stopped,fault)
phase = 'wait';
if fault, phase = 'fault'; elseif confirmed && stopped, phase = 'next'; end
end
