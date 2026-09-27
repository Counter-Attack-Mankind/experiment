function shrink_report = ShrinkWrittenInitialGuessEFForScheme1(matfile)
% ShrinkWrittenInitialGuessEFForScheme1
% Scheme 1 initial-value repair for NLP1 obstacle constraints.
%
% The upstream trajectory and EF screening may still leave some NLP1
% triangle-area obstacle constraints infeasible. This function keeps the
% trajectory unchanged, then synchronously shrinks up/down/left/right and
% A/B/C/D initial values so all obstacle constraints start from the feasible side.

global params

if nargin < 1 || isempty(matfile)
    matfile = 'written_initial_guess_data.mat';
end

opts = getShrinkOptions();
S = load(matfile);
if ~isfield(S, 'data')
    error('No variable named data found in %s.', matfile);
end
D = S.data;

Nbox = numel(D.AX);
scale_used = ones(Nbox, 1);
before_min_slack = inf(Nbox, 1);
after_min_slack = inf(Nbox, 1);
is_repaired = false(Nbox, 1);
is_unresolved = false(Nbox, 1);

fprintf('\n========== Scheme 1 EF Shrink Initial Guess ==========\n');
fprintf('scale_min                 : %.6f\n', opts.scale_min);
fprintf('scale_step                : %.6f\n', opts.scale_step);
fprintf('safety_slack              : %.6e\n', opts.safety_slack);

for i = 1:Nbox
    before_min_slack(i) = intervalObstacleMinSlack(D, i);
    if before_min_slack(i) >= opts.safety_slack
        after_min_slack(i) = before_min_slack(i);
        continue;
    end

    best = [];
    best_any = [];
    scale_grid = 1:-opts.scale_step:opts.scale_min;
    if isempty(scale_grid) || scale_grid(end) > opts.scale_min
        scale_grid = [scale_grid, opts.scale_min];
    end

    for sc = scale_grid
        C = makeScaledInterval(D, i, sc);
        min_slack = intervalObstacleMinSlack(C, i);
        if isempty(best_any) || min_slack > best_any.min_slack
            best_any = struct('scale', sc, 'data', C, 'min_slack', min_slack);
        end
        if min_slack >= opts.safety_slack
            best = struct('scale', sc, 'data', C, 'min_slack', min_slack);
            break;
        end
    end

    if isempty(best)
        best = best_any;
        is_unresolved(i) = best.min_slack < opts.safety_slack;
    end

    D = copyIntervalEF(D, best.data, i);
    scale_used(i) = best.scale;
    after_min_slack(i) = best.min_slack;
    is_repaired(i) = true;
end

data = D; %#ok<NASGU>
save(matfile, 'data');
rewriteIgInival(D);

shrink_report = struct();
shrink_report.scale_used = scale_used;
shrink_report.before_min_slack = before_min_slack;
shrink_report.after_min_slack = after_min_slack;
shrink_report.is_repaired = is_repaired;
shrink_report.is_unresolved = is_unresolved;
shrink_report.num_repaired = nnz(is_repaired);
shrink_report.num_unresolved = nnz(is_unresolved);
shrink_report.min_before_slack = min(before_min_slack);
shrink_report.min_after_slack = min(after_min_slack);
shrink_report.all_collision_constraints_satisfied = all(after_min_slack >= opts.safety_slack | ~isfinite(after_min_slack));
shrink_report.opts = opts;

params.scheme1.shrink_report = shrink_report;

fprintf('repaired EF boxes          : %d / %d\n', shrink_report.num_repaired, Nbox);
fprintf('unresolved EF boxes        : %d / %d\n', shrink_report.num_unresolved, Nbox);
fprintf('min slack before shrink    : %.6e\n', shrink_report.min_before_slack);
fprintf('min slack after shrink     : %.6e\n', shrink_report.min_after_slack);
fprintf('all collision constraints satisfied: %d\n', shrink_report.all_collision_constraints_satisfied);
fprintf('======================================================\n\n');

if opts.show_plot
    PlotShrinkCollisionVerification(D, shrink_report);
end
end

function opts = getShrinkOptions()
global params
opts.scale_min = 0;
opts.scale_step = 0.02;
opts.safety_slack = 1e-7;
opts.show_plot = true;

if isfield(params, 'scheme1') && isfield(params.scheme1, 'shrink')
    user_opts = params.scheme1.shrink;
    names = fieldnames(opts);
    for k = 1:numel(names)
        if isfield(user_opts, names{k})
            opts.(names{k}) = user_opts.(names{k});
        end
    end
end
end

function PlotShrinkCollisionVerification(D, shrink_report)
AX = D.AX(:); AY = D.AY(:);
BX = D.BX(:); BY = D.BY(:);
CX = D.CX(:); CY = D.CY(:);
DX = D.DX(:); DY = D.DY(:);
x = D.x(:); y = D.y(:);
obs = D.meta.obs;

bad_idx = find(shrink_report.after_min_slack < shrink_report.opts.safety_slack & ...
               isfinite(shrink_report.after_min_slack));

figure('Name', 'Scheme 1 Shrink Collision Verification', 'Color', 'w');
hold on; axis equal; box on; grid on;
xlabel('x'); ylabel('y');
title('Scheme 1 shrink result: blue=satisfied, red=violated');

for ii = 1:numel(obs)
    fill(obs(ii).x(:), obs(ii).y(:), [0.82 0.82 0.82], ...
        'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 1.0);
end

plot(x, y, 'k--', 'LineWidth', 1.0);

for i = 1:numel(AX)
    XX = [AX(i), BX(i), CX(i), DX(i), AX(i)];
    YY = [AY(i), BY(i), CY(i), DY(i), AY(i)];

    if ismember(i, bad_idx)
        col = [0.85 0.1 0.1];
        lw = 2.0;
    else
        col = [0.2 0.45 0.9];
        lw = 0.8;
    end

    plot(XX, YY, '-', 'Color', col, 'LineWidth', lw);
end

for k = 1:numel(bad_idx)
    i = bad_idx(k);
    cx = mean([AX(i), BX(i), CX(i), DX(i)]);
    cy = mean([AY(i), BY(i), CY(i), DY(i)]);
    text(cx, cy, sprintf('%d', i), 'Color', [0.85 0.1 0.1], ...
        'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
end

hObs = patch(nan, nan, [0.82 0.82 0.82], 'EdgeColor', [0.3 0.3 0.3]);
hPath = plot(nan, nan, 'k--', 'LineWidth', 1.0);
hOk = plot(nan, nan, '-', 'Color', [0.2 0.45 0.9], 'LineWidth', 1.2);
hBad = plot(nan, nan, '-', 'Color', [0.85 0.1 0.1], 'LineWidth', 2.0);
legend([hObs, hPath, hOk, hBad], {'Obstacle', 'Path', 'Satisfied EF box', 'Violated EF box'}, ...
    'Location', 'bestoutside');

figure('Name', 'Scheme 1 Shrink Collision Binary Status', 'Color', 'w');
status = double(shrink_report.after_min_slack >= shrink_report.opts.safety_slack | ...
                ~isfinite(shrink_report.after_min_slack));
stairs(status, 'LineWidth', 1.5); grid on;
ylim([-0.1, 1.1]);
yticks([0 1]);
yticklabels({'violated', 'satisfied'});
xlabel('interval i');
title('NLP1 collision constraint status after shrink');
end

function C = makeScaledInterval(D, i, sc)
C = D;

C.up(i) = sc * D.up(i);
C.down(i) = sc * D.down(i);
C.left(i) = sc * D.left(i);
C.right(i) = sc * D.right(i);

[C.AX(i), C.AY(i), C.BX(i), C.BY(i), C.CX(i), C.CY(i), C.DX(i), C.DY(i)] = ...
    intervalBoxVertices(C, i);
end

function D = copyIntervalEF(D, C, i)
fields = {'up','down','left','right','AX','AY','BX','BY','CX','CY','DX','DY'};
for k = 1:numel(fields)
    f = fields{k};
    D.(f)(i) = C.(f)(i);
end
end

function [AX, AY, BX, BY, CX, CY, DX, DY] = intervalBoxVertices(D, i)
LF = D.meta.LF;
lr = D.meta.lr;
hlb = D.meta.hlb;

ct = cos(D.theta(i));
st = sin(D.theta(i));

AX = D.x(i) + (LF + D.up(i)) * ct - (hlb + D.left(i)) * st;
BX = D.x(i) + (LF + D.up(i)) * ct + (hlb + D.right(i)) * st;
CX = D.x(i) - (lr + D.down(i)) * ct + (hlb + D.right(i)) * st;
DX = D.x(i) - (lr + D.down(i)) * ct - (hlb + D.left(i)) * st;

AY = D.y(i) + (LF + D.up(i)) * st + (hlb + D.left(i)) * ct;
BY = D.y(i) + (LF + D.up(i)) * st - (hlb + D.right(i)) * ct;
CY = D.y(i) - (lr + D.down(i)) * st - (hlb + D.right(i)) * ct;
DY = D.y(i) - (lr + D.down(i)) * st + (hlb + D.left(i)) * ct;
end

function min_slack = intervalObstacleMinSlack(D, i)
if i < 2 || i > D.meta.Nfe - 1
    min_slack = inf;
    return;
end

ef_poly = [D.AX(i), D.AY(i);
           D.BX(i), D.BY(i);
           D.CX(i), D.CY(i);
           D.DX(i), D.DY(i)];

if any(ef_poly(:,1) < 0) || any(ef_poly(:,1) > 30) || ...
   any(ef_poly(:,2) < 0) || any(ef_poly(:,2) > 30)
    min_slack = -inf;
    return;
end

LF = D.meta.LF;
lr = D.meta.lr;
hlb = D.meta.hlb;
lb = 2 * hlb;
ef_area_rhs = (LF + lr + D.up(i) + D.down(i)) * ...
              (lb + D.right(i) + D.left(i)) + 0.1;

min_slack = inf;
for obs_idx = 1:D.meta.Nobs
    [obs_poly, obs_area_rhs] = getNlpObstacle(D.meta.obs(obs_idx));

    for jj = 1:size(obs_poly, 1)
        area_sum = sumPointToPolygonTriangleAreas(obs_poly(jj, :), ef_poly);
        min_slack = min(min_slack, area_sum - ef_area_rhs);
    end

    for jj = 1:size(ef_poly, 1)
        area_sum = sumPointToPolygonTriangleAreas(ef_poly(jj, :), obs_poly);
        min_slack = min(min_slack, area_sum - obs_area_rhs);
    end
end
end

function [poly, area_rhs] = getNlpObstacle(obs)
ox = obs.x(:);
oy = obs.y(:);

if numel(ox) >= 2 && abs(ox(1) - ox(end)) < 1e-12 && abs(oy(1) - oy(end)) < 1e-12
    ox(end) = [];
    oy(end) = [];
end

nv = numel(ox);
if nv == 3
    ox = [ox; ox(end)];
    oy = [oy; oy(end)];
elseif nv ~= 4
    error('Obstacle has %d vertices; current NLP1 obstacle constraints support triangles/quads only.', nv);
end

poly = [ox, oy];
raw_area = polygonArea(poly);
area_rhs = min(raw_area + 0.02, raw_area * 1.02);
end

function area_sum = sumPointToPolygonTriangleAreas(P, poly)
area_sum = 0;
n = size(poly, 1);
for i = 1:n
    p1 = poly(i, :);
    if i < n
        p2 = poly(i + 1, :);
    else
        p2 = poly(1, :);
    end
    area_sum = area_sum + triangleArea(P, p1, p2);
end
end

function A = triangleArea(p1, p2, p3)
A = 0.5 * abs((p2(1) - p1(1)) * (p3(2) - p1(2)) - ...
              (p2(2) - p1(2)) * (p3(1) - p1(1)));
end

function A = polygonArea(poly)
x = poly(:, 1);
y = poly(:, 2);
A = 0.5 * abs(sum(x .* y([2:end, 1]) - y .* x([2:end, 1])));
end

function rewriteIgInival(D)
fid = fopen('ig.INIVAL', 'w');
if fid < 0
    error('Cannot open ig.INIVAL for writing.');
end
cleanup_obj = onCleanup(@() fclose(fid));

Nfe = D.meta.Nfe;
for ii = 1:Nfe
    fprintf(fid, 'let x[%d] := %.12f;\r\n',     ii, D.x(ii));
    fprintf(fid, 'let y[%d] := %.12f;\r\n',     ii, D.y(ii));
    fprintf(fid, 'let theta[%d] := %.12f;\r\n', ii, D.theta(ii));
    fprintf(fid, 'let v[%d] := %.12f;\r\n',     ii, D.v(ii));
    fprintf(fid, 'let a[%d] := %.12f;\r\n',     ii, D.a(ii));
    fprintf(fid, 'let phy[%d] := %.12f;\r\n',   ii, D.phy(ii));
    fprintf(fid, 'let w[%d] := %.12f;\r\n',     ii, D.w(ii));
end

for ii = 1:(Nfe-1)
    fprintf(fid, 'let k[%d] := %.12f;\r\n',      ii, D.kappa(ii));
    fprintf(fid, 'let dt[%d] := %.12f;\r\n',     ii, D.dt(ii));
    fprintf(fid, 'let s[%d] := %.12f;\r\n',      ii, D.s(ii));
    fprintf(fid, 'let splus[%d] := %.12f;\r\n',  ii, D.splus(ii));
    fprintf(fid, 'let sminus[%d] := %.12f;\r\n', ii, D.sminus(ii));

    fprintf(fid, 'let up[%d] := %.12f;\r\n',     ii, D.up(ii));
    fprintf(fid, 'let down[%d] := %.12f;\r\n',   ii, D.down(ii));
    fprintf(fid, 'let left[%d] := %.12f;\r\n',   ii, D.left(ii));
    fprintf(fid, 'let right[%d] := %.12f;\r\n',  ii, D.right(ii));

    fprintf(fid, 'let AX[%d] := %.12f;\r\n', ii, D.AX(ii));
    fprintf(fid, 'let AY[%d] := %.12f;\r\n', ii, D.AY(ii));
    fprintf(fid, 'let BX[%d] := %.12f;\r\n', ii, D.BX(ii));
    fprintf(fid, 'let BY[%d] := %.12f;\r\n', ii, D.BY(ii));
    fprintf(fid, 'let CX[%d] := %.12f;\r\n', ii, D.CX(ii));
    fprintf(fid, 'let CY[%d] := %.12f;\r\n', ii, D.CY(ii));
    fprintf(fid, 'let DX[%d] := %.12f;\r\n', ii, D.DX(ii));
    fprintf(fid, 'let DY[%d] := %.12f;\r\n', ii, D.DY(ii));
end

fprintf(fid, 'let tf := %.12f;\r\n', D.tf);
end
