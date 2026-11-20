function radar_path_corner_rules_20261112
% Radar grid path -> lane centerline -> corner arcs / heading rotation / speed.
% Offline pre-check only: no hardware, no vehicle motion.
% Output rules feed step 2 (C converter). MATLAB R2016b compatible, ASCII only.
% Run: cd to this folder, then >> radar_path_corner_rules_20261112
% Values marked R come from the rule book, E are engineering candidates (unverified).

root = fileparts(fileparts(mfilename('fullpath')));
outDir = fullfile(root, 'evidence', 'navigation-planning', '2026-11-12', 'corner-rules');
if ~exist(outDir, 'dir'), mkdir(outDir); end
logf = fopen(fullfile(outDir, 'matlab_run.log'), 'w');
say(logf, 'MATLAB %s  %s\n', version, datestr(now, 'yyyy-mm-dd HH:MM:SS'));

%% Parameters
P.field = 2400;            % R FIELD-01
P.grid = 480;              % radar default 5x5 grid, user confirmed unchanged
P.ylo = [550 1400];        % R yellow 450x450, centre cross lane 400
P.yhi = [1000 1850];
P.L = 283; P.W = 254;      % user confirmed body envelope (L along heading)
P.m_hard = 30;             % E margin to yellow / field edge (incl. 20 mm tracking)
P.m_hidden = 10;           % E margin to zone where an obstacle may be merged into a yellow cell
P.m_obst = 20;             % E margin to occupied cells
P.obs_r = 25;              % R obstacle phi50
P.cls_tol = 15;            % E radar cell classification error (calib residual 14 mm)
P.stop_relax = 200;        % within 200 mm of leg start/end only require staying inside field
P.raw_protrude = 85;       % R turntable protrudes 75-85 mm into field, take max
P.vmax = 400;              % E translation peak 40 cm/s (frozen decision 4)
P.Vw = 400;                % E wheel surface speed limit mm/s (~99 RPM, unverified)
P.k = 0.5 * (212 + 206);   % half track + half wheelbase, mm
P.a = 300;                 % E tangential accel mm/s^2 (unverified)
P.alat = 400;              % E corner lateral accel mm/s^2 (unverified)
P.ds = 10;                 % sampling step mm
P.Rgrid = 0:20:480;        % fillet radius candidates
P.rdiag = 0.5 * hypot(P.L, P.W);

ENV = make_env(P);
YELLOW_CELLS = [6 8 16 18];

% Station stop points (current vehicle_tuning.h values) and frozen headings (decision 5)
ST.id = [0 4 2 14 10 22];
ST.name = {'start2', 'start1', 'qr', 'raw', 'coarse', 'storage'};
ST.xy = [150 150; 2250 150; 1200 300; 2160 1200; 240 1200; 1200 2160];
ST.th = [90 90 90 90 -90 180] * pi / 180;

checks = {};

%% A. Grid cells versus lanes / yellow areas
yfrac = zeros(25, 1); ctype = cell(25, 1);
for id = 0:24
    b = cellbox(id, P); ar = 0;
    for k = 1:4, ar = ar + rect_inter(b, ENV.yellow(k, :)); end
    yfrac(id + 1) = ar / P.grid^2;
    ctype{id + 1} = celltype(id);
end
cellTable = table((0:24)', ctype, yfrac, 'VariableNames', {'cell', 'type', 'yellow_fraction'});
writetable(cellTable, fullfile(outDir, 'cell_geometry.csv'));
expY = 410^2 / 480^2; expC = 2 * 40 * 410 / 480^2; expM = 4 * 40 * 40 / 480^2;
checks(end + 1, :) = {'A1 yellow cells 6/8/16/18 are 72.96% yellow (70 mm gray L-strip outside)', ...
    all(abs(yfrac(YELLOW_CELLS + 1) - expY) < 1e-9), sprintf('%.4f', yfrac(9))};
checks(end + 1, :) = {'A2 centre-lane cells 7/11/13/17 contain two 40 mm yellow slivers', ...
    all(abs(yfrac([7 11 13 17] + 1) - expC) < 1e-9), sprintf('%.4f', yfrac(8))};
checks(end + 1, :) = {'A3 centre cell 12 contains four 40x40 yellow corners', ...
    abs(yfrac(13) - expM) < 1e-9, sprintf('%.4f', yfrac(13))};
others = setdiff(0:24, [YELLOW_CELLS 7 11 13 17 12]);
checks(end + 1, :) = {'A4 all other cells have no yellow', all(yfrac(others + 1) == 0), ''};
okMap = true;
for i = 1:numel(ST.id), okMap = okMap && point_cell(ST.xy(i, :), P) == ST.id(i); end
checks(end + 1, :) = {'A5 every station stop point lies in its expected radar cell', okMap, ''};
say(logf, 'A: yellow cell frac %.4f, centre-lane frac %.4f, centre frac %.4f\n', expY, expC, expM);

%% B1. Fixed-heading corner fillet radius per node type
reps = {'corner', 0, [0 -1], [1 0]; 'edgeT', 2, [1 0], [0 1]; 'center', 12, [0 1], [-1 0]};
cornerRows = {};
RULE.types = {'corner', 'edgeT', 'center'};
RULE.Rrec = zeros(1, 3); RULE.vc = zeros(1, 3);
ENV0 = ENV; ENV0.obst = zeros(0, 4);
for t = 1:3
    [nx, ny] = cellc(reps{t, 2}, P); N = [nx ny];
    d1 = reps{t, 3}; d2 = reps{t, 4};
    Rbest = inf;
    for hAx = [pi / 2 0]
        Rmax = NaN; mins = [NaN NaN];
        for R = P.Rgrid
            Tq = build_traj([N - 960 * d1; N; N + 960 * d2], [0; R; 0], P.ds);
            E = eval_traj(Tq, hAx * ones(numel(Tq.x), 1), ENV0, P, false);
            if ~E.pass, break; end
            Rmax = R; mins = E.min([1 3]);
        end
        cornerRows(end + 1, :) = {RULE.types{t}, reps{t, 2}, hAx * 180 / pi, Rmax, mins(1), mins(2)}; %#ok<AGROW>
        if isnan(Rmax), Rbest = NaN; else, Rbest = min(Rbest, Rmax); end
    end
    RULE.Rrec(t) = Rbest;
    RULE.vc(t) = min(P.vmax, sqrt(P.alat * max(Rbest, 0)));
    checks(end + 1, :) = {sprintf('B1 %s turn feasible at R=0 (fixed heading)', RULE.types{t}), ...
        ~isnan(Rbest), sprintf('Rrec=%g mm, v_corner=%.0f mm/s', Rbest, RULE.vc(t))}; %#ok<AGROW>
    say(logf, 'B1 %-7s Rrec=%5g mm  v_corner=%4.0f mm/s\n', RULE.types{t}, Rbest, RULE.vc(t));
end
cornerTable = cell2table(cornerRows, 'VariableNames', ...
    {'type', 'node', 'heading_deg', 'Rmax_mm', 'min_yellow_mm', 'min_hidden_mm'});
writetable(cornerTable, fullfile(outDir, 'corner_rules.csv'));

%% B2. Where may the car rotate (full 360 deg sweep, circle of radius rdiag)
names = {'node0_corner', 'node2_edgeT', 'node12_center', 'cell1_band_mid', 'cell7_centrelane_mid', ...
    'start2', 'start1', 'qr', 'raw', 'coarse', 'storage'};
pts = [240 240; 1200 240; 1200 1200; 720 240; 1200 720; ST.xy];
rotRows = {};
for i = 1:size(pts, 1)
    [ok, m] = rot_safe(pts(i, 1), pts(i, 2), ENV0, P);
    stn = inf;
    for a = linspace(0, pi, 91)
        v = eval_pose(pts(i, 1), pts(i, 2), a, ENV0, P); stn = min(stn, v(5));
    end
    rotRows(end + 1, :) = {names{i}, pts(i, 1), pts(i, 2), ok, m(1), m(2), m(3), stn}; %#ok<AGROW>
end
rotTable = cell2table(rotRows, 'VariableNames', {'point', 'x', 'y', 'rotation_safe', ...
    'yellow_margin', 'field_margin', 'hidden_margin', 'station_clear_info'});
writetable(rotTable, fullfile(outDir, 'rotation_points.csv'));
checks(end + 1, :) = {'B2 centre-lane mid-span is NOT rotation safe', ~rotTable.rotation_safe(5), ...
    sprintf('yellow margin %.1f mm', rotTable.yellow_margin(5))};
checks(end + 1, :) = {'B2 node 12 centre is rotation safe', rotTable.rotation_safe(3), ...
    sprintf('yellow margin %.1f mm', rotTable.yellow_margin(3))};

%% B3. Station stop poses: clearance to station areas (information, pre-existing stop values)
stopRows = {};
for i = 1:numel(ST.id)
    v = eval_pose(ST.xy(i, 1), ST.xy(i, 2), ST.th(i), ENV0, P);
    stopRows(end + 1, :) = {ST.name{i}, ST.xy(i, 1), ST.xy(i, 2), ST.th(i) * 180 / pi, v(2), v(5)}; %#ok<AGROW>
    say(logf, 'B3 stop %-8s field %.1f  station %.1f mm\n', ST.name{i}, v(2), v(5));
end
stopTable = cell2table(stopRows, 'VariableNames', {'stop', 'x', 'y', 'heading_deg', ...
    'field_clear', 'station_clear_info'});
writetable(stopTable, fullfile(outDir, 'stop_poses.csv'));

%% C. Match legs on several obstacle maps
maps = {'M0_empty', []; 'M1_scan511', [4 12]; 'M2', 12; 'M3', [7 13]; 'M4', [1 19]; 'M5', [3 11 21]};
legRows = {};
allOkM0 = true; failOther = 0; ratioOk = true; headOk = true;
matchTime = zeros(size(maps, 1), 2);
for mi = 1:size(maps, 1)
    occ = maps{mi, 2};
    E1 = ENV; E1.obst = zeros(numel(occ), 4);
    for k = 1:numel(occ)
        E1.obst(k, :) = cellbox(occ(k), P) + (P.obs_r + P.cls_tol) * [-1 1 -1 1];
    end
    fig = figure('Visible', 'off', 'Position', [50 50 1400 700]);
    for si = 1:2
        S = [0 4]; S = S(si);
        legs = [S 2; 2 14; 14 10; 10 22; 22 14; 22 S];
        ax = subplot(1, 2, si); hold(ax, 'on'); draw_field(ax, E1, P, occ);
        title(ax, sprintf('%s  start cell %d', strrep(maps{mi, 1}, '_', ' '), S));
        legT = nan(size(legs, 1), 1);
        cols = lines(size(legs, 1));
        for li = 1:size(legs, 1)
            Rl = run_leg(legs(li, 1), legs(li, 2), E1, P, ST, RULE, occ);
            legRows(end + 1, :) = {maps{mi, 1}, S, li, legs(li, 1), legs(li, 2), Rl.status, ...
                num2str(Rl.cells), num2str(Rl.wp), Rl.dth_deg, Rl.scale, Rl.len, Rl.time, ...
                Rl.min(1), Rl.min(2), Rl.min(3), Rl.min(4), Rl.min(5), Rl.ratio}; %#ok<AGROW>
            if strcmp(Rl.status, 'ok')
                legT(li) = Rl.time;
                ratioOk = ratioOk && Rl.ratio <= 1 + 1e-6;
                headOk = headOk && Rl.head_err < 1e-6;
                plot(ax, Rl.T.x, Rl.T.y, '-', 'Color', cols(li, :), 'LineWidth', 1.5);
                stepIdx = 1:max(1, round(150 / P.ds)):numel(Rl.T.x);
                for q = stepIdx
                    C = carpoly(Rl.T.x(q), Rl.T.y(q), Rl.th(q), P);
                    plot(ax, C([1:4 1], 1), C([1:4 1], 2), '-', 'Color', [cols(li, :) 0.25]);
                end
            end
            if mi == 1 && ~strcmp(Rl.status, 'ok'), allOkM0 = false; end
            if mi > 1 && ~any(strcmp(Rl.status, {'ok', 'start_or_goal_occupied', 'no_path'}))
                failOther = failOther + 1;
            end
            say(logf, 'C %-10s S%d leg%d %2d->%2d %-24s wp[%s] t=%.1fs minY=%.0f minH=%.0f\n', ...
                maps{mi, 1}, S, li, legs(li, 1), legs(li, 2), Rl.status, num2str(Rl.wp), ...
                Rl.time, Rl.min(1), Rl.min(3));
        end
        % Whole-match travel order: S-qr, qr-raw, raw-coarse, coarse-storage, storage-raw,
        % raw-coarse, coarse-storage, storage-S
        order = [1 2 3 4 5 3 4 6];
        matchTime(mi, si) = sum(legT(order));
    end
    print(fig, fullfile(outDir, sprintf('legs_%s.png', maps{mi, 1})), '-dpng', '-r110');
    close(fig);
end
legTable = cell2table(legRows, 'VariableNames', {'map', 'start_cell', 'leg', 'from', 'to', ...
    'status', 'cells', 'radar_like_waypoints', 'dtheta_deg', 'R_scale', 'length_mm', 'time_s', ...
    'min_yellow', 'min_field', 'min_hidden', 'min_obst', 'min_station_info', 'wheel_ratio'});
writetable(legTable, fullfile(outDir, 'legs.csv'));
checks(end + 1, :) = {'C1 all 12 legs pass on empty map M0', allOkM0, ''};
checks(end + 1, :) = {'C2 no clearance/rotation failure on obstacle maps M1-M5', failOther == 0, ...
    sprintf('%d failures', failOther)};
checks(end + 1, :) = {'C3 wheel demand within limit on all ok legs', ratioOk, ''};
checks(end + 1, :) = {'C4 final heading equals station heading on all ok legs', headOk, ''};
mt = array2table(matchTime, 'VariableNames', {'start0_s', 'start4_s'}, 'RowNames', maps(:, 1));
writetable(mt, fullfile(outDir, 'match_travel_time.csv'), 'WriteRowNames', true);

% Known radar result (scan511, obstacles 4/12): 0 -> 24 returned waypoints [20 24]
blocked = false(25, 1); blocked([YELLOW_CELLS 4 12] + 1) = true;
[cells024, ok024] = plan_grid(0, 24, blocked);
wp024 = radar_waypoints(cells024);
checks(end + 1, :) = {'C5 local grid planner reproduces the one real radar path 0->20->24', ...
    ok024 && isequal(wp024, [20 24]), num2str(wp024)};

%% D. Rotation-safe map figure
fig = figure('Visible', 'off', 'Position', [50 50 800 760]);
ax = axes(fig); hold(ax, 'on'); draw_field(ax, ENV0, P, []);
[gx, gy] = meshgrid(5:10:P.field - 5, 5:10:P.field - 5);
okg = rot_safe(gx(:), gy(:), ENV0, P);
plot(ax, gx(okg), gy(okg), '.', 'Color', [0.2 0.7 0.2], 'MarkerSize', 3);
title(ax, 'green: car centre where a full rotation keeps all margins');
print(fig, fullfile(outDir, 'rotation_safe_map.png'), '-dpng', '-r110');
close(fig);

%% Summary
nPass = sum(cell2mat(checks(:, 2))); nAll = size(checks, 1);
for i = 1:nAll
    say(logf, '[%s] %s %s\n', pf(checks{i, 2}), checks{i, 1}, checks{i, 3});
end
rules.params = P; rules.types = RULE.types; rules.Rrec_mm = RULE.Rrec; rules.v_corner_mm_s = RULE.vc;
rules.rotation_rule = 'heading may change only where a circle of radius rdiag keeps all margins';
rules.lane_centerline = 'radar cell centres (240/1200/2160 mm)';
writejson(fullfile(outDir, 'rules.json'), rules);
report.checks_pass = nPass; report.checks_total = nAll;
report.failed = checks(~cell2mat(checks(:, 2)), 1);
report.match_travel_time_s = matchTime;
writejson(fullfile(outDir, 'summary.json'), report);
say(logf, 'RESULT: %s (%d/%d checks)\n', pf(nPass == nAll), nPass, nAll);
fclose(logf);
end

%% ---------------- leg runner ----------------
function R = run_leg(sid, gid, ENV, P, ST, RULE, occ)
R.status = ''; R.cells = []; R.wp = []; R.dth_deg = NaN; R.scale = NaN; R.len = NaN;
R.time = NaN; R.min = nan(1, 5); R.ratio = NaN; R.head_err = NaN; R.T = []; R.th = [];
blocked = false(25, 1); blocked([6 8 16 18] + 1) = true; blocked(occ + 1) = true;
if blocked(sid + 1) || blocked(gid + 1), R.status = 'start_or_goal_occupied'; return; end
[cells, ok] = plan_grid(sid, gid, blocked);
if ~ok, R.status = 'no_path'; return; end
R.cells = cells; R.wp = radar_waypoints(cells);
[ps, ths] = stop_of(sid, ST, P); [pg, thg] = stop_of(gid, ST, P);
V = build_vertices(cells, ps, pg, P);
Rv = zeros(size(V, 1), 1);
for j = 2:size(V, 1) - 1
    id = point_cell(V(j, :), P); [cx, cy] = cellc(id, P);
    t = find(strcmp(RULE.types, celltype(id)));
    if ~isempty(t) && norm(V(j, :) - [cx cy]) < 1, Rv(j) = RULE.Rrec(t); end
end
d = wrap(thg - ths);
if abs(abs(d) - pi) < 1e-6, cands = [pi -pi]; else, cands = d; end
R.status = 'fail_clearance'; bestT = inf;
for sc = [1 0.5 0]
    T = build_traj(V, Rv * sc, P.ds);
    safe = rot_safe(T.x, T.y, ENV, P);
    for dd = cands
        [th, hp, okA] = schedule(T, safe, ths, dd);
        if ~okA
            if strcmp(R.status, 'fail_clearance'), R.status = 'fail_no_rotation_zone'; end
            continue;
        end
        E = eval_traj(T, th, ENV, P, true);
        if ~E.pass
            if any(isnan(R.min)), R.min = E.min; end
            continue;
        end
        [tt, ratio] = speed_profile(T, th, hp, P);
        if tt < bestT
            bestT = tt; R.status = 'ok'; R.dth_deg = dd * 180 / pi; R.scale = sc;
            R.len = T.s(end); R.time = tt; R.min = E.min; R.ratio = ratio;
            R.head_err = abs(wrap(th(end) - thg)); R.T = T; R.th = th;
        end
    end
    if strcmp(R.status, 'ok'), break; end
end
end

function [p, th] = stop_of(id, ST, P)
k = find(ST.id == id, 1);
if isempty(k), [x, y] = cellc(id, P); p = [x y]; th = pi / 2;
else, p = ST.xy(k, :); th = ST.th(k); end
end

%% ---------------- environment ----------------
function ENV = make_env(P)
ENV.yellow = zeros(4, 4); k = 0;
for i = 1:2
    for j = 1:2
        k = k + 1; ENV.yellow(k, :) = [P.ylo(i) P.yhi(i) P.ylo(j) P.yhi(j)];
    end
end
% An obstacle centred in the 70 mm gray strip of a yellow cell (or within cls_tol of it)
% is reported as that permanent cell, i.e. invisible to path planning.
ex = P.ylo(1) - (P.grid - P.cls_tol - P.obs_r);
ENV.hidden = ENV.yellow;
for k = 1:4
    b = ENV.yellow(k, :);
    if b(1) < P.field / 2, b(1) = b(1) - ex; else, b(2) = b(2) + ex; end
    if b(3) < P.field / 2, b(3) = b(3) - ex; else, b(4) = b(4) + ex; end
    ENV.hidden(k, :) = b;
end
ENV.station = [0 150 910 1490; 910 1490 2250 2400];        % coarse, storage (580x150)
ENV.table = [P.field + 150 - P.raw_protrude, 1200, 150];   % raw turntable circle
ENV.obst = zeros(0, 4);
end

%% ---------------- grid helpers ----------------
function [col, row] = cellcr(id)
row = floor(id / 5); col = mod(id, 5);
if mod(row, 2) == 1, col = 4 - col; end
end
function id = cellid(col, row)
if mod(row, 2) == 1, id = row * 5 + (4 - col); else, id = row * 5 + col; end
end
function [x, y] = cellc(id, P)
[col, row] = cellcr(id); x = (col + 0.5) * P.grid; y = (row + 0.5) * P.grid;
end
function b = cellbox(id, P)
[col, row] = cellcr(id); b = [col (col + 1) row (row + 1)] * P.grid;
end
function id = point_cell(p, P)
col = min(4, max(0, floor(p(1) / P.grid))); row = min(4, max(0, floor(p(2) / P.grid)));
id = cellid(col, row);
end
function t = celltype(id)
if any(id == [6 8 16 18]), t = 'yellow';
elseif any(id == [0 4 20 24]), t = 'corner';
elseif any(id == [2 10 14 22]), t = 'edgeT';
elseif id == 12, t = 'center';
elseif any(id == [7 11 13 17]), t = 'centre_lane';
else, t = 'band'; end
end
function a = rect_inter(b, c)
w = min(b(2), c(2)) - max(b(1), c(1)); h = min(b(4), c(4)) - max(b(3), c(3));
a = max(w, 0) * max(h, 0);
end

function [cells, ok] = plan_grid(s, g, blocked)
% Dijkstra on (cell, direction): 1 per move + 0.001 per turn (shortest, then fewest turns).
% NOTE: stand-in for the radar planner; real legs must be requested from the radar.
dirs = [1 0; -1 0; 0 1; 0 -1]; BIG = 1e9;
D = BIG * ones(25, 4); prv = zeros(25, 4, 2); done = false(25, 4);
D(s + 1, :) = 0;
while true
    tmp = D; tmp(done) = BIG; [m, idx] = min(tmp(:));
    if m >= BIG, break; end
    [ci, di] = ind2sub([25 4], idx); done(ci, di) = true;
    [c, r] = cellcr(ci - 1);
    for nd = 1:4
        nc = c + dirs(nd, 1); nr = r + dirs(nd, 2);
        if nc < 0 || nc > 4 || nr < 0 || nr > 4, continue; end
        nid = cellid(nc, nr);
        if blocked(nid + 1), continue; end
        cost = m + 1 + 0.001 * (nd ~= di);
        if cost < D(nid + 1, nd) - 1e-12
            D(nid + 1, nd) = cost; prv(nid + 1, nd, :) = [ci di];
        end
    end
end
[m, dg] = min(D(g + 1, :));
if m >= BIG, cells = []; ok = false; return; end
ci = g + 1; di = dg; seq = g;
while prv(ci, di, 1) ~= 0
    p = squeeze(prv(ci, di, :)); ci = p(1); di = p(2); seq(end + 1) = ci - 1; %#ok<AGROW>
end
cells = fliplr(seq); ok = cells(1) == s;
end

function wp = radar_waypoints(cells)
% Radar PATH format: turning cells plus goal, start excluded.
wp = [];
if numel(cells) < 2, wp = cells(end); return; end
for i = 2:numel(cells) - 1
    [c0, r0] = cellcr(cells(i - 1)); [c1, r1] = cellcr(cells(i)); [c2, r2] = cellcr(cells(i + 1));
    if ~isequal([c1 - c0, r1 - r0], [c2 - c1, r2 - r1]), wp(end + 1) = cells(i); end %#ok<AGROW>
end
wp(end + 1) = cells(end);
end

function V = build_vertices(cells, ps, pg, P)
n = numel(cells); C = zeros(n, 2);
for i = 1:n, [C(i, 1), C(i, 2)] = cellc(cells(i), P); end
if n == 1, V = dedupe([ps; pg]); return; end
keep = 1;
for i = 2:n - 1
    d1 = C(i, :) - C(i - 1, :); d2 = C(i + 1, :) - C(i, :);
    if abs(d1(1) * d2(2) - d1(2) * d2(1)) > 1e-9, keep(end + 1) = i; end %#ok<AGROW>
end
keep(end + 1) = n;
Lp = C(keep, :);
u1 = sign(C(2, :) - C(1, :));
if u1(2) == 0, eS = [ps(1) C(1, 2)]; else, eS = [C(1, 1) ps(2)]; end
u2 = sign(C(n, :) - C(n - 1, :));
if u2(2) == 0, eG = [pg(1) C(n, 2)]; else, eG = [C(n, 1) pg(2)]; end
Lp(1, :) = eS; Lp(end, :) = eG;
V = dedupe([ps; Lp; pg]);
end

function V = dedupe(V)
k = [true; sqrt(sum(diff(V).^2, 2)) > 1e-6]; V = V(k, :);
i = 2;
while i < size(V, 1)
    a = V(i, :) - V(i - 1, :); b = V(i + 1, :) - V(i, :);
    if abs(a(1) * b(2) - a(2) * b(1)) < 1e-6 && dot(a, b) > 0, V(i, :) = []; else, i = i + 1; end
end
end

%% ---------------- trajectory ----------------
function T = build_traj(V, R, ds)
m = size(V, 1); u = zeros(m - 1, 2); len = zeros(m - 1, 1);
for i = 1:m - 1
    dv = V(i + 1, :) - V(i, :); len(i) = norm(dv); u(i, :) = dv / len(i);
end
Tn = zeros(m, 1); gam = zeros(m, 1); sg = zeros(m, 1); R = R(:);
for j = 2:m - 1
    c = max(-1, min(1, dot(u(j - 1, :), u(j, :)))); gam(j) = acos(c);
    sg(j) = sign(u(j - 1, 1) * u(j, 2) - u(j - 1, 2) * u(j, 1));
    if gam(j) > pi - 1e-6 || gam(j) < 1e-9, R(j) = 0; end
    Tn(j) = R(j) * tan(gam(j) / 2);
    lim = min(0.5 * len(j - 1), 0.5 * len(j));
    if Tn(j) > lim, Tn(j) = lim; R(j) = lim / tan(gam(j) / 2); end
end
X = []; Y = []; PSI = []; K = []; STP = [];
for i = 1:m - 1
    p0 = V(i, :) + Tn(i) * u(i, :); p1 = V(i + 1, :) - Tn(i + 1) * u(i, :);
    n = max(1, ceil(norm(p1 - p0) / ds)); t = (0:n - 1)' / n;
    X = [X; p0(1) + t * (p1(1) - p0(1))]; Y = [Y; p0(2) + t * (p1(2) - p0(2))]; %#ok<AGROW>
    PSI = [PSI; repmat(atan2(u(i, 2), u(i, 1)), n, 1)]; K = [K; zeros(n, 1)]; %#ok<AGROW>
    st = zeros(n, 1);
    if i > 1 && R(i) == 0 && gam(i) > 1e-9, st(1) = 1; end   % sharp corner: stop
    STP = [STP; st]; %#ok<AGROW>
    if i < m - 1
        j = i + 1;
        if gam(j) > 1e-9 && R(j) > 0
            c0 = p1 + R(j) * sg(j) * [-u(i, 2) u(i, 1)];
            a0 = atan2(p1(2) - c0(2), p1(1) - c0(1));
            sw = sg(j) * gam(j); na = max(2, ceil(abs(sw) * R(j) / ds)); f = (0:na - 1)' / na;
            X = [X; c0(1) + R(j) * cos(a0 + sw * f)]; Y = [Y; c0(2) + R(j) * sin(a0 + sw * f)]; %#ok<AGROW>
            PSI = [PSI; atan2(u(i, 2), u(i, 1)) + sw * f]; K = [K; ones(na, 1) / R(j)]; %#ok<AGROW>
            STP = [STP; zeros(na, 1)]; %#ok<AGROW>
        end
    end
end
X = [X; V(m, 1)]; Y = [Y; V(m, 2)]; PSI = [PSI; atan2(u(m - 1, 2), u(m - 1, 1))];
K = [K; 0]; STP = [STP; 1]; STP(1) = 1;
T.x = X; T.y = Y; T.psi = PSI; T.k = K; T.stop = STP; T.R = R;
T.s = [0; cumsum(hypot(diff(X), diff(Y)))];
end

function [th, hp, ok] = schedule(T, safe, th0, d)
% Spread heading change over samples where full rotation is safe.
n = numel(T.x); ds = diff(T.s); allow = safe(1:end - 1) & safe(2:end) & ds > 0;
w = ds .* allow; W = sum(w);
th = th0 * ones(n, 1); hp = zeros(n, 1); ok = true;
if d == 0, return; end
if W <= 0, ok = false; return; end
th = th0 + d * [0; cumsum(w)] / W;
q = ds > 0; hp(q) = (d * w(q) / W) ./ ds(q);
end

function [tt, ratio] = speed_profile(T, th, hp, P)
n = numel(T.x); vl = P.vmax * ones(n, 1);
for i = 1:n
    if T.k(i) > 0, vl(i) = min(vl(i), sqrt(P.alat / T.k(i))); end
    al = T.psi(i) - th(i);
    vl(i) = min(vl(i), P.Vw / (abs(cos(al)) + abs(sin(al)) + P.k * abs(hp(i))));
    if T.stop(i), vl(i) = 0; end
end
v = vl; ds = diff(T.s);
for i = 2:n, v(i) = min(v(i), sqrt(v(i - 1)^2 + 2 * P.a * ds(i - 1))); end
for i = n - 1:-1:1, v(i) = min(v(i), sqrt(v(i + 1)^2 + 2 * P.a * ds(i))); end
vm = 0.5 * (v(1:end - 1) + v(2:end)); q = ds > 0;
tt = sum(ds(q) ./ max(vm(q), 1e-6));
al = T.psi - th;
ratio = max((v .* (abs(cos(al)) + abs(sin(al))) + P.k * abs(v .* hp)) / P.Vw);
end

%% ---------------- clearance ----------------
function E = eval_traj(T, th, ENV, P, relax)
n = numel(T.x); Vv = zeros(n, 5);
for i = 1:n, Vv(i, :) = eval_pose(T.x(i), T.y(i), th(i), ENV, P); end
req = P.m_hard * ones(n, 1);
if relax
    S = T.s(end); req(T.s <= P.stop_relax | T.s >= S - P.stop_relax) = 0;
end
okv = Vv(:, 1) >= P.m_hard & Vv(:, 2) >= req & Vv(:, 3) >= P.m_hidden & Vv(:, 4) >= P.m_obst;
E.pass = all(okv); E.min = min(Vv, [], 1);
end

function v = eval_pose(x, y, th, ENV, P)
C = carpoly(x, y, th, P);
yel = inf; hid = inf; obs = inf; stn = inf;
for k = 1:4
    yel = min(yel, poly_box(C, ENV.yellow(k, :)));
    hid = min(hid, poly_box(C, ENV.hidden(k, :)));
end
for k = 1:size(ENV.obst, 1), obs = min(obs, poly_box(C, ENV.obst(k, :))); end
for k = 1:size(ENV.station, 1), stn = min(stn, poly_box(C, ENV.station(k, :))); end
stn = min(stn, pt_poly(ENV.table(1:2), C) - ENV.table(3));
fld = min([C(:, 1); P.field - C(:, 1); C(:, 2); P.field - C(:, 2)]);
v = [yel fld hid obs stn];
end

function C = carpoly(x, y, th, P)
loc = [P.L P.W; -P.L P.W; -P.L -P.W; P.L -P.W] / 2;
Rm = [cos(th) -sin(th); sin(th) cos(th)];
C = (Rm * loc')' + repmat([x y], 4, 1);
end

function [ok, m] = rot_safe(x, y, ENV, P)
x = x(:); y = y(:); r = P.rdiag;
yel = inf(size(x)); hid = inf(size(x)); obs = inf(size(x));
for k = 1:4
    yel = min(yel, pbox(x, y, ENV.yellow(k, :))); hid = min(hid, pbox(x, y, ENV.hidden(k, :)));
end
for k = 1:size(ENV.obst, 1), obs = min(obs, pbox(x, y, ENV.obst(k, :))); end
fld = min(min(x, P.field - x), min(y, P.field - y));
m = [yel fld hid] - r;
ok = (yel - r >= P.m_hard) & (fld - r >= P.m_hard) & (hid - r >= P.m_hidden) & (obs - r >= P.m_obst);
end

function d = pbox(x, y, b)
dx = max(max(b(1) - x, 0), x - b(2)); dy = max(max(b(3) - y, 0), y - b(4));
d = hypot(dx, dy);
d(x >= b(1) & x <= b(2) & y >= b(3) & y <= b(4)) = -1;
end

function d = poly_box(C, b)
B = [b(1) b(3); b(2) b(3); b(2) b(4); b(1) b(4)];
[sep, depth] = sat(C, B);
if ~sep, d = -depth; return; end
d = inf;
for i = 1:4, d = min([d, pt_edges(C(i, :), B), pt_edges(B(i, :), C)]); end
end

function [sep, depth] = sat(A, B)
ax = [edge_normals(A); edge_normals(B)]; sep = false; depth = inf;
for i = 1:size(ax, 1)
    pa = A * ax(i, :)'; pb = B * ax(i, :)';
    ov = min(max(pa), max(pb)) - max(min(pa), min(pb));
    if ov <= 0, sep = true; depth = 0; return; end
    depth = min(depth, ov);
end
end

function N = edge_normals(A)
Ed = A([2:end 1], :) - A; N = [-Ed(:, 2) Ed(:, 1)];
N = N ./ repmat(hypot(N(:, 1), N(:, 2)), 1, 2);
end

function d = pt_edges(p, A)
d = inf; n = size(A, 1);
for i = 1:n
    a = A(i, :); b = A(mod(i, n) + 1, :); ab = b - a;
    t = max(0, min(1, dot(p - a, ab) / dot(ab, ab)));
    d = min(d, norm(p - (a + t * ab)));
end
end

function d = pt_poly(p, C)
d = pt_edges(p, C);
if inpolygon(p(1), p(2), C(:, 1), C(:, 2)), d = -d; end
end

%% ---------------- misc ----------------
function a = wrap(a)
a = mod(a + pi, 2 * pi) - pi;
end

function draw_field(ax, ENV, P, occ)
plot(ax, [0 P.field P.field 0 0], [0 0 P.field P.field 0], 'k-');
for k = 1:4
    b = ENV.yellow(k, :);
    fill(ax, b([1 2 2 1]), b([3 3 4 4]), [1 0.95 0.6], 'EdgeColor', 'none');
    h = ENV.hidden(k, :);
    plot(ax, h([1 2 2 1 1]), h([3 3 4 4 3]), ':', 'Color', [0.8 0.5 0]);
end
for k = 1:size(ENV.station, 1)
    b = ENV.station(k, :);
    fill(ax, b([1 2 2 1]), b([3 3 4 4]), [0.75 0.85 1], 'EdgeColor', 'b');
end
a = linspace(0, 2 * pi, 60);
plot(ax, ENV.table(1) + ENV.table(3) * cos(a), ENV.table(2) + ENV.table(3) * sin(a), 'b-');
fill(ax, [0 300 300 0], [0 0 300 300], [0.6 0.7 1], 'EdgeColor', 'none');
fill(ax, [2100 2400 2400 2100], [0 0 300 300], [0.6 0.7 1], 'EdgeColor', 'none');
for g = P.grid:P.grid:P.field - P.grid
    plot(ax, [g g], [0 P.field], '-', 'Color', [0.85 0.85 0.85]);
    plot(ax, [0 P.field], [g g], '-', 'Color', [0.85 0.85 0.85]);
end
for id = 0:24
    [x, y] = cellc(id, P);
    text(ax, x - 200, y + 200, num2str(id), 'Color', [0.5 0.5 0.5], 'FontSize', 7);
end
for k = 1:numel(occ)
    b = cellbox(occ(k), P);
    fill(ax, b([1 2 2 1]), b([3 3 4 4]), [0.3 0.3 0.3], 'FaceAlpha', 0.35, 'EdgeColor', 'k');
end
axis(ax, 'equal'); xlim(ax, [-50 P.field + 150]); ylim(ax, [-50 P.field + 50]);
xlabel(ax, 'X mm (start2 -> start1)'); ylabel(ax, 'Y mm (start2 -> coarse)');
end

function writejson(file, s)
fid = fopen(file, 'w'); fwrite(fid, jsonencode(s), 'char'); fclose(fid);
end

function s = pf(b)
if b, s = 'PASS'; else, s = 'FAIL'; end
end

function say(fid, varargin)
fprintf(varargin{:}); fprintf(fid, varargin{:});
end
