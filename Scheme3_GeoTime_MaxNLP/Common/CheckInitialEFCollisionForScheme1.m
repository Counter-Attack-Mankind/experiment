function report = CheckInitialEFCollisionForScheme1(x, y, theta, v, phy, time, show_plot)
% CheckInitialEFCollisionForScheme1
% Diagnose whether the Scheme 1 initial guess satisfies the NLP-style
% embodied-footprint obstacle constraints before writing ig.INIVAL.

global params

alpha = 60;
tol = 1e-9;
if nargin < 7 || isempty(show_plot)
    show_plot = true;
end

x = x(:);
y = y(:);
theta = theta(:);
v = v(:);
phy = phy(:);
dt = time(:);

Nfe = numel(x);
assert(numel(y) == Nfe, 'y length must match x length.');
assert(numel(theta) == Nfe, 'theta length must match x length.');
assert(numel(v) == Nfe, 'v length must match x length.');
assert(numel(phy) == Nfe, 'phy length must match x length.');
assert(numel(dt) == Nfe - 1, 'time length must be Nfe-1.');

% Match WriteEFInitialGuess/NLP boundary convention.
v(1) = 0;
phy(1) = 0;
v(Nfe) = 0;
phy(Nfe) = 0;

lw = params.vehicle.lw;
LF = params.vehicle.LF;
lr = params.vehicle.lr;
hlb = params.vehicle.hlb;
lb = 2 * hlb;

kappa = tan(phy(1:Nfe-1)) / lw;
s = v(1:Nfe-1) .* dt;
splus = softplus(alpha * s) / alpha;
sminus = softplus(alpha * (-s)) / alpha;

up = splus + hlb .* abs(kappa) .* splus;
down = sminus + hlb .* abs(kappa) .* sminus;

left = smoothMax(alpha, -lr .* kappa .* splus, (LF + 0.5 .* splus) .* kappa .* splus) + ...
       smoothMax(alpha, -LF .* kappa .* sminus, (lr + 0.5 .* sminus) .* kappa .* sminus);
right = smoothMax(alpha, lr .* kappa .* splus, -(LF + 0.5 .* splus) .* kappa .* splus) + ...
        smoothMax(alpha, LF .* kappa .* sminus, -(lr + 0.5 .* sminus) .* kappa .* sminus);

Nbox = Nfe - 1;
AX = zeros(Nbox,1); AY = zeros(Nbox,1);
BX = zeros(Nbox,1); BY = zeros(Nbox,1);
CX = zeros(Nbox,1); CY = zeros(Nbox,1);
DX = zeros(Nbox,1); DY = zeros(Nbox,1);

for i = 1:Nbox
    ct = cos(theta(i));
    st = sin(theta(i));

    AX(i) = x(i) + (LF + up(i)) * ct - (hlb + left(i)) * st;
    BX(i) = x(i) + (LF + up(i)) * ct + (hlb + right(i)) * st;
    CX(i) = x(i) - (lr + down(i)) * ct + (hlb + right(i)) * st;
    DX(i) = x(i) - (lr + down(i)) * ct - (hlb + left(i)) * st;

    AY(i) = y(i) + (LF + up(i)) * st + (hlb + left(i)) * ct;
    BY(i) = y(i) + (LF + up(i)) * st - (hlb + right(i)) * ct;
    CY(i) = y(i) - (lr + down(i)) * st - (hlb + right(i)) * ct;
    DY(i) = y(i) - (lr + down(i)) * st + (hlb + left(i)) * ct;
end

items = struct('box_idx', {}, 'obs_idx', {}, 'constraint', {}, ...
               'vertex_idx', {}, 'violation', {}, 'slack', {});
slacks = [];
bad_idx = false(Nbox, 1);

for i = 2:Nbox
    ef_poly = [AX(i), AY(i);
               BX(i), BY(i);
               CX(i), CY(i);
               DX(i), DY(i)];
    ef_area = (LF + lr + up(i) + down(i)) * (lb + right(i) + left(i));

    for nn = 1:params.environment.num_obs
        [obs_poly, area_nn] = getNlpObstacle(nn);

        for jj = 1:4
            P = obs_poly(jj,:);
            area_sum = sumPointToQuadTriangleAreas(P, ef_poly);
            rhs = ef_area + 0.1;
            slack = area_sum - rhs;
            slacks(end+1,1) = slack; %#ok<AGROW>
            if slack < -tol
                bad_idx(i) = true;
                items(end+1) = makeItem(i, nn, 'obs_vertex_outside_ef_box', jj, -slack, slack); %#ok<AGROW>
            end
        end

        for vertex_name = ["A", "B", "C", "D"]
            switch vertex_name
                case "A", P = ef_poly(1,:);
                case "B", P = ef_poly(2,:);
                case "C", P = ef_poly(3,:);
                case "D", P = ef_poly(4,:);
            end
            area_sum = sumPointToQuadTriangleAreas(P, obs_poly);
            slack = area_sum - area_nn;
            slacks(end+1,1) = slack; %#ok<AGROW>
            if slack < -tol
                bad_idx(i) = true;
                items(end+1) = makeItem(i, nn, ['ef_vertex_', char(vertex_name), '_outside_obstacle'], NaN, -slack, slack); %#ok<AGROW>
            end
        end
    end
end

report = struct();
report.has_collision = any(bad_idx);
report.bad_idx = find(bad_idx);
report.num_bad = numel(report.bad_idx);
report.min_slack = minOrInf(slacks);
report.items = items;
report.alpha = alpha;
report.Nfe = Nfe;
report.Nbox = Nbox;
report.path.x = x;
report.path.y = y;
report.path.theta = theta;
report.box.AX = AX; report.box.AY = AY;
report.box.BX = BX; report.box.BY = BY;
report.box.CX = CX; report.box.CY = CY;
report.box.DX = DX; report.box.DY = DY;
report.ef.s = s;
report.ef.splus = splus;
report.ef.sminus = sminus;
report.ef.up = up;
report.ef.down = down;
report.ef.left = left;
report.ef.right = right;
report.ef.kappa = kappa;

fprintf('\n========== Scheme 1 Initial EF Collision Check ==========\n');
fprintf('Nfe                         : %d\n', Nfe);
fprintf('checked EF boxes             : %d (NLP collision constraints use 2..Nfe-1)\n', Nbox);
fprintf('bad EF box count             : %d\n', report.num_bad);
fprintf('min collision slack          : %.6e\n', report.min_slack);
if report.has_collision
    preview = report.bad_idx(1:min(20, numel(report.bad_idx)));
    fprintf('first bad box indices         : ');
    fprintf('%d ', preview);
    fprintf('\n');
else
    fprintf('initial EF obstacle constraints are satisfied.\n');
end
fprintf('========================================================\n\n');

if show_plot
    PlotScheme1InitialEFBoxes(report);
end
end

function y = softplus(z)
y = max(z, 0) + log1p(exp(-abs(z)));
end

function y = smoothMax(alpha, a, b)
m = max(a, b);
y = m + log(exp(alpha .* (a - m)) + exp(alpha .* (b - m))) / alpha;
end

function [poly, area_nn] = getNlpObstacle(obs_idx)
global params
ox = params.environment.obs(obs_idx).x(:);
oy = params.environment.obs(obs_idx).y(:);

if numel(ox) >= 2 && abs(ox(1) - ox(end)) < 1e-12 && abs(oy(1) - oy(end)) < 1e-12
    ox(end) = [];
    oy(end) = [];
end

nv = numel(ox);
if nv == 3
    ox = [ox; ox(end)];
    oy = [oy; oy(end)];
elseif nv ~= 4
    error('Obstacle %d has %d vertices; only triangles/quads match current NLP.', obs_idx, nv);
end

poly = [ox(:), oy(:)];
raw_area = polygonArea(poly);
area_nn = min(raw_area + 0.02, raw_area * 1.02);
end

function area_sum = sumPointToQuadTriangleAreas(P, quad)
area_sum = 0;
for e = 1:4
    p1 = quad(e,:);
    if e < 4
        p2 = quad(e+1,:);
    else
        p2 = quad(1,:);
    end
    area_sum = area_sum + triangleArea(P, p1, p2);
end
end

function A = triangleArea(p1, p2, p3)
A = 0.5 * abs((p1(1)-p3(1))*(p2(2)-p3(2)) - (p1(2)-p3(2))*(p2(1)-p3(1)));
end

function A = polygonArea(poly)
x = poly(:,1);
y = poly(:,2);
if x(1) ~= x(end) || y(1) ~= y(end)
    x = [x; x(1)];
    y = [y; y(1)];
end
A = 0.5 * abs(sum(x(1:end-1).*y(2:end) - y(1:end-1).*x(2:end)));
end

function item = makeItem(box_idx, obs_idx, constraint, vertex_idx, violation, slack)
item = struct('box_idx', box_idx, 'obs_idx', obs_idx, ...
              'constraint', constraint, 'vertex_idx', vertex_idx, ...
              'violation', violation, 'slack', slack);
end

function v = minOrInf(x)
if isempty(x)
    v = inf;
else
    v = min(x);
end
end

function PlotScheme1InitialEFBoxes(report)
global params

fig = figure('Name', 'Scheme 1 Initial EF Collision Check', 'Color', 'w');
ax = axes(fig);
hold(ax, 'on');
box(ax, 'on');
grid(ax, 'minor');
axis(ax, 'equal');

if isfield(params.environment, 'xmin') && isfield(params.environment, 'xmax') && ...
        isfield(params.environment, 'ymin') && isfield(params.environment, 'ymax')
    axis(ax, [params.environment.xmin, params.environment.xmax, ...
              params.environment.ymin, params.environment.ymax]);
end

if isfield(params.environment, 'obs') && ~isempty(params.environment.obs)
    for ii = 1:numel(params.environment.obs)
        ox = params.environment.obs(ii).x(:);
        oy = params.environment.obs(ii).y(:);
        patch(ax, ox, oy, [0.80 0.80 0.80], ...
            'EdgeColor', [0.35 0.35 0.35], ...
            'LineWidth', 0.8, ...
            'HandleVisibility', 'off');
    end
end

plot(ax, report.path.x, report.path.y, 'k--', 'LineWidth', 1.0, ...
    'DisplayName', 'Initial path');
plot(ax, report.path.x, report.path.y, 'ko', 'MarkerSize', 3, ...
    'MarkerFaceColor', 'k', 'HandleVisibility', 'off');

bad_mask = false(report.Nbox, 1);
bad_mask(report.bad_idx) = true;

normal_edge = [0.10 0.35 0.85];
normal_face = [0.75 0.86 1.00];
bad_edge = [0.90 0.05 0.05];
bad_face = [1.00 0.72 0.72];

h_normal = plot(ax, nan, nan, '-', 'Color', normal_edge, 'LineWidth', 1.0, ...
    'DisplayName', 'Feasible EF box');
h_bad = plot(ax, nan, nan, '-', 'Color', bad_edge, 'LineWidth', 2.0, ...
    'DisplayName', 'Colliding EF box');

for i = 1:report.Nbox
    X = [report.box.AX(i), report.box.BX(i), report.box.CX(i), report.box.DX(i), report.box.AX(i)];
    Y = [report.box.AY(i), report.box.BY(i), report.box.CY(i), report.box.DY(i), report.box.AY(i)];

    if bad_mask(i)
        patch(ax, X, Y, bad_face, ...
            'FaceAlpha', 0.28, ...
            'EdgeColor', bad_edge, ...
            'LineWidth', 2.0, ...
            'HandleVisibility', 'off');
        text(ax, mean(X(1:4)), mean(Y(1:4)), sprintf('%d', i), ...
            'Color', bad_edge, ...
            'FontSize', 9, ...
            'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center');
    else
        patch(ax, X, Y, normal_face, ...
            'FaceAlpha', 0.08, ...
            'EdgeColor', normal_edge, ...
            'LineWidth', 0.8, ...
            'HandleVisibility', 'off');
    end
end

xlabel(ax, 'x / m');
ylabel(ax, 'y / m');
title(ax, sprintf('Scheme 1 Initial EF Boxes: %d bad / %d total', report.num_bad, report.Nbox));
legend(ax, [h_normal, h_bad], 'Location', 'eastoutside');
drawnow;
end
