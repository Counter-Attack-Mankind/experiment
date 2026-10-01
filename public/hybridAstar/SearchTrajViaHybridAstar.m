function success = SearchTrajViaHybridAstar()

success = 0;
global params openlist_ grid_space_ ha_debug_

%% =========================
% 1. Guiding path
% =========================
SearchGuidingPath();
EstimateprogressAlongGuidingLine();

ha_debug_.last_invalid = {};
ha_debug_.max_invalid_keep = 5;
ha_debug_.last_cur_node = [];
ha_debug_.last_cur_path_x = [];
ha_debug_.last_cur_path_y = [];

if params.ha.enable_debug_plot
    InitHADebugFigure();
end

%% =========================
% 2. Initialize Hybrid A*
% =========================
grid_space_ = cell(params.ha.nx, params.ha.ny, params.ha.ntheta);
openlist_ = [];

init_node.x = params.task.x0;
init_node.y = params.task.y0;
init_node.theta = params.task.theta0;
init_node.v = 0;
init_node.phy = 0;
init_node.g = 0;
init_node.h = CalculateH(init_node);
init_node.f = init_node.g + init_node.h;
init_node.parent_id = [-1 -1 -1];
init_node.from_parent = [];
init_node.local_dt = 0;
init_node.is_closed = 0;
init_node.is_open = 1;

init_id = CalculateNodeIndex(init_node);
grid_space_{init_id(1), init_id(2), init_id(3)} = init_node;
openlist_ = [init_node.f, init_id];

goal_node.x = params.task.xf;
goal_node.y = params.task.yf;
goal_node.theta = params.task.thetaf;
goal_id = CalculateNodeIndex(goal_node);

expansion_pattern = SpecifySamplePattern();

completeness_flag = 0;
cur_best_node = init_node;
cur_best_node_cost_val = inf;

iter = 0;
search_tic = tic;
fail_reason = '';

%% =========================
% 3. Hybrid A* search
% =========================
while ~isempty(openlist_)

    iter = iter + 1;

    if iter > params.ha.max_iter
        fail_reason = '达到最大迭代次数';
        break;
    end

    if toc(search_tic) > params.ha.max_search_time
        fail_reason = '搜索时间超过上限';
        break;
    end

    [cur_node, cur_node_id] = ExtractMinFNodeFromOpenlist();

    if params.ha.enable_debug_plot && mod(iter, params.ha.debug_plot_stride) == 0
        ha_debug_.last_cur_node = cur_node;
        [ha_debug_.last_cur_path_x, ha_debug_.last_cur_path_y] = TraceNodePath(cur_node);
        UpdateHADebugFigure(iter, cur_node, cur_best_node, completeness_flag);
        drawnow limitrate;
    end

    parent_phy = cur_node.phy;

    for ii = 1:size(expansion_pattern, 1)

        child_node.v = expansion_pattern(ii, 1);
        child_node.phy = expansion_pattern(ii, 2);

        local_dt = params.ha.simu_unit_duration;
        path_seg = SimulateForward_test(cur_node, child_node.v, child_node.phy, local_dt);

        child_node.x = path_seg.x(end);
        child_node.y = path_seg.y(end);
        child_node.theta = path_seg.theta(end);
        child_node.parent_id = cur_node_id;
        child_node.from_parent = path_seg;
        child_node.local_dt = local_dt;

        child_node_id = CalculateNodeIndex(child_node);

        % 已关闭节点不再扩展
        if ~isempty(grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)})
            if grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)}.is_closed
                continue;
            end
        end

        % 仅使用真实车身进行碰撞检测
        if ~IsPathValid(path_seg, child_node.phy, child_node.v, local_dt)

            if params.ha.enable_debug_plot
                dbg.x = child_node.x;
                dbg.y = child_node.y;
                dbg.theta = child_node.theta;
                dbg.phy = child_node.phy;
                dbg.dir = child_node.v;
                dbg.path_seg = path_seg;

                ha_debug_.last_invalid{end+1} = dbg;

                if numel(ha_debug_.last_invalid) > ha_debug_.max_invalid_keep
                    ha_debug_.last_invalid = ha_debug_.last_invalid(end-ha_debug_.max_invalid_keep+1:end);
                end
            end

            continue;
        end

        %% -------------------------
        % Cost
        % --------------------------
        penalty_phy = abs(child_node.phy - parent_phy) * params.ha.penalty_on_phy_change;

        if child_node.v < 0
            penalty_backward = local_dt * params.ha.penalty_for_backward;
        else
            penalty_backward = 0;
        end

        child_candidate_g = cur_node.g + local_dt + penalty_phy + penalty_backward;

        %% -------------------------
        % New node
        % --------------------------
        if isempty(grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)})

            child_node.g = child_candidate_g;
            child_node.h = CalculateH(child_node);
            child_node.f = child_node.g + child_node.h;
            child_node.is_closed = 0;
            child_node.is_open = 1;

            grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)} = child_node;
            openlist_ = [openlist_; child_node.f, child_node_id];

        else

            %% -------------------------
            % Existing open node
            % --------------------------
            old_node = grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)};

            if old_node.is_open && child_candidate_g < old_node.g

                child_node.g = child_candidate_g;
                child_node.h = old_node.h;
                child_node.f = child_node.g + child_node.h;
                child_node.is_closed = 0;
                child_node.is_open = 1;

                grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)} = child_node;
                UpdateOpenlist(child_node, child_node_id);
            end
        end

        %% -------------------------
        % Goal condition
        % --------------------------
        if all(child_node_id == goal_id)
            completeness_flag = 1;
            cur_node = child_node;
            break;
        end

        %% -------------------------
        % Debug: closest node
        % --------------------------
        pos_error = hypot(child_node.x - params.task.xf, child_node.y - params.task.yf);
        theta_error = abs(atan2(sin(child_node.theta - params.task.thetaf), cos(child_node.theta - params.task.thetaf)));
        cost_val_candidate = pos_error + theta_error;

        if cost_val_candidate < cur_best_node_cost_val
            cur_best_node_cost_val = cost_val_candidate;
            cur_best_node = child_node;
        end
    end

    if completeness_flag
        break;
    end
end

%% =========================
% 4. Search result
% =========================
if ~completeness_flag

    if isempty(fail_reason)
        if isempty(openlist_)
            fail_reason = 'openlist 已耗尽，未找到可行解';
        else
            fail_reason = '未知搜索失败';
        end
    end

    fprintf('\nHybrid A* 搜索失败：%s\n', fail_reason);

    params.ha.success = 0;
    params.ha.fail_reason = fail_reason;
    return;
end

if params.ha.enable_debug_plot
    UpdateHADebugFigure(iter, cur_node, cur_best_node, completeness_flag);
    DrawHAFinalMarks(cur_best_node, completeness_flag);
    drawnow;
end

%% =========================
% 5. Backtracking
% =========================
Traj.x = [];
Traj.y = [];
Traj.theta = [];
Traj.v = [];
Traj.phy = [];

while true

    path_seg = cur_node.from_parent;

    if isempty(path_seg)
        break;
    end

    nseg = numel(path_seg.x);

    Traj.x = [path_seg.x, Traj.x];
    Traj.y = [path_seg.y, Traj.y];
    Traj.theta = [path_seg.theta, Traj.theta];
    Traj.v = [ones(1, nseg) * cur_node.v, Traj.v];
    Traj.phy = [ones(1, nseg) * cur_node.phy, Traj.phy];

    parent_id = cur_node.parent_id;
    cur_node = grid_space_{parent_id(1), parent_id(2), parent_id(3)};
end

x = Traj.x;
y = Traj.y;
theta = Traj.theta;
v_traj = Traj.v;
phy_traj = Traj.phy;

if isempty(x)
    error('Hybrid A*: reconstructed trajectory is empty.');
end

%% =========================
% 6. Append exact terminal pose
% =========================
% 搜索终止条件使用目标离散格。
% 这里保留精确任务终点，供后续初值生成使用。
%
% 注意：不再进行 RS analytic expansion。

if hypot(x(end) - params.task.xf, y(end) - params.task.yf) > 1e-10 || ...
        abs(atan2(sin(theta(end) - params.task.thetaf), cos(theta(end) - params.task.thetaf))) > 1e-10

    x = [x, params.task.xf];
    y = [y, params.task.yf];
    theta = [theta, params.task.thetaf];

    if isempty(v_traj)
        v_traj = 1;
        phy_traj = 0;
    else
        v_traj = [v_traj, v_traj(end)];
        phy_traj = [phy_traj, phy_traj(end)];
    end
end

%% =========================
% 7. Remove duplicated points
% =========================
keep = true(1, numel(x));

for i = 2:numel(x)
    if hypot(x(i) - x(i-1), y(i) - y(i-1)) < 1e-10 && ...
            abs(atan2(sin(theta(i) - theta(i-1)), cos(theta(i) - theta(i-1)))) < 1e-10
        keep(i) = false;
    end
end

x = x(keep);
y = y(keep);
theta = theta(keep);
v_traj = v_traj(keep);
phy_traj = phy_traj(keep);

%% =========================
% 8. Unwrap heading
% =========================
for i = 2:numel(theta)

    while theta(i) - theta(i-1) > pi
        theta(i) = theta(i) - 2*pi;
    end

    while theta(i) - theta(i-1) < -pi
        theta(i) = theta(i) + 2*pi;
    end
end

%% =========================
% 9. Direction changes
% =========================
change_idx = find(v_traj(2:end) ~= v_traj(1:end-1)) + 1;

params.change = change_idx;

fprintf('\n========== Hybrid A* ==========\n');
fprintf('iterations       : %d\n', iter);
fprintf('trajectory points: %d\n', numel(x));
fprintf('direction changes: %d\n', numel(change_idx));
fprintf('================================\n\n');

%% =========================
% 10. Save result
% =========================
params.ha.x = x;
params.ha.y = y;
params.ha.theta = theta;
params.ha.v = v_traj;
params.ha.phy = phy_traj;
params.ha.success = 1;
params.ha.fail_reason = '';

success = 1;

end

function EstimateprogressAlongGuidingLine()
global params

s = zeros(1, length(params.guiding_path.x));

for ii = 2:length(params.guiding_path.x)
    s(ii) = s(ii-1) + hypot(params.guiding_path.x(ii) - params.guiding_path.x(ii-1), ...
                            params.guiding_path.y(ii) - params.guiding_path.y(ii-1));
end

params.progress_s = s(end) - s;
end

function idx = CalculateNodeIndex(node)
global params

params.ha_res_x = params.environment.xhorizon / params.ha.nx;
params.ha_res_y = params.environment.yhorizon / params.ha.ny;
params.ha_res_theta = 2*pi / params.ha.ntheta;

ind1 = ceil((node.x - params.environment.xmin) / params.ha_res_x) + 1;
ind2 = ceil((node.y - params.environment.ymin) / params.ha_res_y) + 1;
ind3 = ceil(RegulateAngle(node.theta) / params.ha_res_theta) + 1;

ind1 = max(1, min(ind1, params.ha.nx));
ind2 = max(1, min(ind2, params.ha.ny));
ind3 = max(1, min(ind3, params.ha.ntheta));

idx = [ind1, ind2, ind3];
end

function expansion_pattern = SpecifySamplePattern()
global params

phy_list = linspace(-params.vehicle.phy_max, params.vehicle.phy_max, params.ha.num_phy_ha);

expansion_pattern = [];

for ii = 1:params.ha.num_phy_ha
    expansion_pattern = [expansion_pattern; 1, phy_list(ii)];
    expansion_pattern = [expansion_pattern; -1, phy_list(ii)];
end
end

function [cur_node, cur_node_id] = ExtractMinFNodeFromOpenlist()

global grid_space_ openlist_

[~, id] = min(openlist_(:,1));

cur_node_id = openlist_(id,2:4);
cur_node = grid_space_{cur_node_id(1), cur_node_id(2), cur_node_id(3)};

openlist_(id,:) = [];

grid_space_{cur_node_id(1), cur_node_id(2), cur_node_id(3)}.is_closed = 1;
grid_space_{cur_node_id(1), cur_node_id(2), cur_node_id(3)}.is_open = 0;

end

%%
function UpdateOpenlist(child_node, child_node_id)

global openlist_

mask = openlist_(:,2) == child_node_id(1) & ...
       openlist_(:,3) == child_node_id(2) & ...
       openlist_(:,4) == child_node_id(3);

openlist_(mask,:) = [];

openlist_ = [openlist_; child_node.f, child_node_id];

end

%%
function is_valid = IsPathValid(path_seg, phy, dir, local_dt)

is_valid = IsVehicleBodyValid(path_seg.x, path_seg.y, path_seg.theta, phy, dir, local_dt);

end

%%
%% 碰撞检测: 车辆矩形 + 障碍物（SAT）
function is_valid = IsVehicleBodyValid(x, y, theta, phy, dir, local_dt)
is_valid = 0;
global params

% 若传入的是向量，这里统一按列向量处理
x     = x(:);
y     = y(:);
theta = theta(:);

% ===== 当前/离散车身矩形检测 =====
for kk = 1:length(x)
    body_poly = GetVehicleRectangle(x(kk), y(kk), theta(kk));

    % 边界检测
    if any(body_poly(:,1) > params.environment.xmax) || ...
       any(body_poly(:,1) < params.environment.xmin) || ...
       any(body_poly(:,2) > params.environment.ymax) || ...
       any(body_poly(:,2) < params.environment.ymin)
        return;
    end

    % 障碍物碰撞检测
    for ii = 1:params.environment.num_obs
        obs_poly = [params.environment.obs(ii).x(:), ...
                    params.environment.obs(ii).y(:)];

        if SAT_PolygonCollision(body_poly, obs_poly)
            return;
        end
    end
end
is_valid = 1;

end
function is_collide = SAT_PolygonCollision(poly1, poly2)
% poly1, poly2: N×2, M×2
% 返回 true 表示碰撞，false 表示无碰撞

is_collide = true;

axes = [GetAxesFromPolygon(poly1);
        GetAxesFromPolygon(poly2)];

for k = 1:size(axes,1)
    axis_k = axes(k,:);

    nrm = norm(axis_k);
    if nrm < 1e-12
        continue;
    end
    axis_k = axis_k / nrm;

    proj1 = poly1 * axis_k';
    proj2 = poly2 * axis_k';

    if max(proj1) < min(proj2) || max(proj2) < min(proj1)
        is_collide = false;
        return;
    end
end
end
%%
function axes = GetAxesFromPolygon(poly)
n = size(poly,1);
axes = zeros(n,2);

for i = 1:n
    p1 = poly(i,:);
    if i < n
        p2 = poly(i+1,:);
    else
        p2 = poly(1,:);
    end

    edge = p2 - p1;
    normal = [-edge(2), edge(1)];
    axes(i,:) = normal;
end
end
%% 启发式函数
function val = CalculateH(child_node)
global params
list = hypot(params.guiding_path.x - child_node.x, params.guiding_path.y - child_node.y); % 当前节点到引导线每点距离
ind = find(list == min(list));
ind = ind(end);  % 取最近点索引
val = params.ha.multiplier_on_heuristics * (params.progress_s(ind) + 0.1 * min(list)); % 启发式 = 剩余距离 + 微小偏差
end
%% (初始化A*调试界面)
function InitHADebugFigure()
global params

figure('Name', 'Hybrid A* Debug View', 'Color', 'w');
clf;
hold on; box on; grid on; axis equal;

xlabel('x');
ylabel('y');
title('Hybrid A* Search Debug');

xlim([params.environment.xmin, params.environment.xmax]);
ylim([params.environment.ymin, params.environment.ymax]);

% 画障碍物
for ii = 1:params.environment.num_obs
    fill(params.environment.obs(ii).x, params.environment.obs(ii).y, ...
        [0.7 0.7 0.7], 'EdgeColor', 'k', 'FaceAlpha', 0.5);
end

% 起点终点
plot(params.task.x0, params.task.y0, 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(params.task.xf, params.task.yf, 'ro', 'MarkerSize', 10, 'LineWidth', 2);

% 引导线
if isfield(params, 'guiding_path') && isfield(params.guiding_path, 'x') && ~isempty(params.guiding_path.x)
    plot(params.guiding_path.x, params.guiding_path.y, 'b--', 'LineWidth', 1.0);
end

legend('Obstacle', 'Start', 'Goal', 'Guiding path');
end
%%
function [x_path, y_path] = TraceNodePath(node)
global grid_space_

x_path = [];
y_path = [];

cur_node = node;

while 1
    if ~isfield(cur_node, 'from_parent') || isempty(cur_node.from_parent)
        break;
    end

    seg = cur_node.from_parent;
    x_path = [seg.x, x_path];
    y_path = [seg.y, y_path];

    pid = cur_node.parent_id;
    if any(pid < 1)
        break;
    end
    cur_node = grid_space_{pid(1), pid(2), pid(3)};
end
end
