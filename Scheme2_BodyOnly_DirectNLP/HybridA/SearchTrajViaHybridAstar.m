function success = SearchTrajViaHybridAstar()
success = 0;

% 主函数：使用Hybrid A*搜索初始可行轨迹
global params openlist_ grid_space_ ha_debug_

%   params: 参数配置（车辆、环境、搜索等）
%   openlist_: 当前A*的待扩展节点列表
%   grid_space_: 离散状态空间，用于记录每个离散格点的节点信息

% ------------------------------正式搜索------------------------------
SearchGuidingPath();    % 搜索一条引导线（简单A*），不考虑车辆动力学与姿态
EstimateprogressAlongGuidingLine();  % 计算每个引导点沿线的累计距离，用作启发式
ha_debug_.last_invalid = {};
ha_debug_.max_invalid_keep = 5;
ha_debug_.last_cur_node = [];
ha_debug_.last_cur_path_x = [];
ha_debug_.last_cur_path_y = [];


% ===== 调试图初始化 =====
if params.ha.enable_debug_plot
    InitHADebugFigure();
end

% 初始化3D离散网格空间 (x, y, θ)
grid_space_ = cell(params.ha.nx, params.ha.ny, params.ha.ntheta);

% 初始化起点节点
init_node.x = params.task.x0;        % 起点x
init_node.y = params.task.y0;        % 起点y
init_node.theta = params.task.theta0;% 起点theta

init_node.h = CalculateH(init_node); % 启发式值
init_node.g = 0;                      % 从起点到自身代价为0
init_node.f = init_node.g + init_node.h; % f = g + h
init_node.parent_id = [-1 -1 -1];    % 父节点索引（-1表示无父节点）
init_node.v = 1;                      % 当前速度
init_node.phy = 0;                    % 当前转向角
init_node.from_parent = [];           % 存储从父节点来的路径段
init_node.is_closed = 0;              % 是否已扩展
init_node.is_open = 1;                % 是否在openlist中

index = CalculateNodeIndex(init_node);   % 将连续状态映射到离散格点索引
grid_space_{index(1), index(2), index(3)} = init_node; % 存储节点
openlist_ = [init_node.f, index];       % 初始化openlist

% ===== 【新增】：目标索引与完成标志 =====
goal_node.x = params.task.xf;
goal_node.y = params.task.yf;
goal_node.theta = params.task.thetaf;
goal_id = CalculateNodeIndex(goal_node);   % 目标离散索引

completeness_flag = 0;         % 是否完成
complete_via_rs_flag = 0;      % 是否通过RS完成
rs_seg = [];                   % 保存成功的RS段（用于最后拼接）
counter = 0;                   % RS尝试计数器（模仿第一版）
%=============================================

% 获取扩展模式
expansion_pattern = SpecifySamplePattern(); % 生成所有可能的v, phy组合
cur_best_node = init_node;                 % 当前最优节点初始化为起点
cur_best_node_cost_val = Inf;             % 当前最优成本初始化为无穷
iter = 0;                                 % 迭代次数初始化

% ===== 搜索失败退出条件 =====
search_tic = tic;
fail_reason = '';


% --------------------主循环--------------------
while (~isempty(openlist_))
    iter = iter + 1;  %更新迭代次数

    % ===== 失败退出条件 =====
    if iter > params.ha.max_iter
        fail_reason = '达到最大迭代次数';
        break;
    end
    if size(openlist_, 1) > params.ha.max_openlist
        fail_reason = 'openlist 超过上限';
        break;
    end
    if toc(search_tic) > params.ha.max_search_time
        fail_reason = '搜索时间超过上限';
        break;
    end
    %=============================
    [cur_node, cur_node_id] = ExtractMinFNodeFromOpenlist(); % 取f值最小节点

    % 实时调试显示
    if params.ha.enable_debug_plot && mod(iter, params.ha.debug_plot_stride) == 0
        ha_debug_.last_cur_node = cur_node;
        [ha_debug_.last_cur_path_x, ha_debug_.last_cur_path_y] = TraceNodePath(cur_node);
        UpdateHADebugFigure(iter, cur_node, cur_best_node, completeness_flag);
        drawnow limitrate;
    end


    parent_v = cur_node.v;                % 父节点速度
    parent_phy = cur_node.phy;            % 父节点转向

    % 遍历所有扩展模式
    for ii = 1 : size(expansion_pattern, 1)
        child_node.v = expansion_pattern(ii, 1);   % 子节点速度
        child_node.phy = expansion_pattern(ii, 2); % 子节点转向

        % 前向模拟轨迹段（长度 = simu_unit_duration）
        local_dt = params.ha.simu_unit_duration;
        path_seg = SimulateForward_test(cur_node, child_node.v, child_node.phy, local_dt);

        % 取轨迹段末端作为子节点状态
        child_node.x = path_seg.x(end);
        child_node.y = path_seg.y(end);
        child_node.theta = path_seg.theta(end);
        child_node.parent_id = cur_node_id;      % 设置父节点
        child_node.from_parent = path_seg;       % 保存从父节点来的路径段
        child_node.local_dt = local_dt;
        child_node_id = CalculateNodeIndex(child_node); % 映射到离散格点

        % -------------------冲突检测: 已闭节点-----------------
        if ((~isempty(grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)})) && ...
            (grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)}.is_closed == 1))

            continue;  % 如果该格点已扩展，则跳过
        end

        % -------------------新节点-----------------
        if (isempty(grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)}))
            % 如果该格点从未被探索
            if IsPathValid(path_seg, child_node.phy, child_node.v, local_dt)  % 检查路径是否有效（碰撞检测）
                if child_node.v < 0
                    penalty_on_backward = local_dt * params.ha.penalty_for_backward; % 倒车惩罚：先采用单步惩罚，避免累计倒车距离计算错误
                else
                    penalty_on_backward = 0;
                end
                penalty_on_phy_change = abs(child_node.phy - parent_phy) * params.ha.penalty_on_phy_change; % 
                penalty_on_dir_change = abs(child_node.v - parent_v) * params.ha.penalty_on_direction_change;

                %【更改】惩罚加和
                step_cost = local_dt + penalty_on_phy_change + penalty_on_dir_change + penalty_on_backward;
                child_node.g = cur_node.g + step_cost;  % 计算累计代价
                child_node.h = CalculateH(child_node);      % 计算启发式
                child_node.f = child_node.h + child_node.g; % f = g + h


                % 可视化路径段
                if (params.ha.enable_recording)
                    PlotPathSegment(path_seg, iter);
                    h2 = text(0.5, 29.5, ['Iter: ', num2str(iter)], 'FontSize', 28);
                    writeVideo(vidObj, getframe(gcf));
                    delete(h2);
                end

                child_node.is_closed = 0;   % 标记为未闭
                child_node.is_open = 1;     % 标记为在openlist
                grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)} = child_node; % 保存节点
                openlist_ = [openlist_; child_node.f, child_node_id];  % 加入openlist

                % ===== Step3 【新增】：命中终点目标格子则结束 =====
                if all(child_node_id == goal_id)
                    completeness_flag = 1;
                    cur_node = child_node;      % 回溯就从这个节点开始
                    break;                      % 跳出for
                end

                % ===== Step3 【新增】：周期性尝试 RS 直连终点 =====
                distance = hypot(child_node.x - params.task.xf, child_node.y - params.task.yf);
                N = 0.8 * distance - 3;                 % 与第一版一致的经验触发
                counter = counter + 1;

                if counter > N
                    counter = 0;
                    [rs_seg_try, rs_len] = GenerateRsPathSeg([child_node.x, child_node.y, child_node.theta], ...
                                                     [params.task.xf, params.task.yf, params.task.thetaf]);

                    % RS 段是否全程无碰撞
                    if ~isempty(rs_seg_try) && IsRsPathValid(rs_seg_try)
                        completeness_flag = 1;
                        complete_via_rs_flag = 1;      % 标记为通过 RS 完成
                        rs_seg = rs_seg_try;           % 保存完整 RS 段
                        cur_node = child_node;         % 回溯从这个节点开始
                        break;                          % 退出 for 循环，跳出搜索
                    end
                end
                %===========================================================
            else
                    % ===== 记录最近的失败候选，供调试可视化 =====
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

                % 如果路径无效，直接标记关闭
                continue;
            end
        end
        
        % -------------------已经在openlist的节点-----------------
        if ((~isempty(grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)})) && ...
             (grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)}.is_open))
            
            if child_node.v < 0
                penalty_on_backward = local_dt * params.ha.penalty_for_backward;
            else
                penalty_on_backward = 0;
            end
            % 计算角度变换的惩罚
            penalty_on_phy_change = abs(child_node.phy - parent_phy) * params.ha.penalty_on_phy_change;

            %【新增】换向惩罚
            penalty_on_dir_change = abs(child_node.v - parent_v) * params.ha.penalty_on_direction_change;

            %惩罚加和
            step_cost = local_dt + penalty_on_phy_change + penalty_on_dir_change + penalty_on_backward;

            % 【更改】candidate_g 候选g，对"已存在节点"的新路径尝试后，加和代价
            child_candidate_g = cur_node.g + step_cost;

            child_previous_g = grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)}.g;
            if (child_previous_g > child_candidate_g + 0.1)    % 新路径更优
                child_node.h = grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)}.h;

                % 【新增】更新g值
                child_node.g = child_candidate_g;
                child_node.f = child_node.h + child_node.g;

                child_node.is_closed = 0;
                child_node.is_open = 1;

                %【新增】父信息与路径段：必须更新，否则回溯会回溯到旧父节点
                child_node.parent_id   = cur_node_id;
                child_node.from_parent = path_seg;

                grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)} = child_node;
                UpdateOpenlist(child_node, child_node_id);   % 更新openlist
            end
        end

        % -------------------记录最接近终点的节点-----------------
        cost_val_candidate = max(...
            hypot(child_node.x - params.task.xf, child_node.y - params.task.yf), ... % 空间距离
            50 * (max(sin(child_node.theta) - sin(params.task.thetaf), ...            % 姿态差
                      cos(child_node.theta) - cos(params.task.thetaf))) ...
            );
        if (cost_val_candidate < cur_best_node_cost_val)
            cur_best_node_cost_val = cost_val_candidate;
            cur_best_node = child_node;
        end
    end


    if completeness_flag
        break;                      % 跳出while
    end
end

if ~completeness_flag && isempty(fail_reason)
    if isempty(openlist_)
        fail_reason = 'openlist 已耗尽，未找到可行解';
    else
        fail_reason = '未知原因失败';
    end
end

if ~completeness_flag
    fprintf('\nHybrid A* 搜索失败：%s\n', fail_reason);
    params.ha.success = 0;
    params.ha.fail_reason = fail_reason;
    return;
end

% ===== 搜索结束后的最终调试显示 =====
if params.ha.enable_debug_plot
    UpdateHADebugFigure(iter, cur_node, cur_best_node, completeness_flag);
    DrawHAFinalMarks(cur_best_node, completeness_flag);
    drawnow;
end

% --------------------回溯路径--------------------
Traj.x = [];
Traj.y = [];
Traj.theta = [];
Traj.v = [];
Traj.phy = [];
Traj.local_dt = [];

while (1)
    path_seg = cur_node.from_parent;
    if isempty(path_seg)
        break;
    end

    nseg = numel(path_seg.x);

    v_seg = ones(1, nseg) * cur_node.v;
    phy_seg = ones(1, nseg) * cur_node.phy;
    dt_seg = ones(1, nseg) * cur_node.local_dt;

    Traj.x = [path_seg.x, Traj.x];
    Traj.y = [path_seg.y, Traj.y];
    Traj.theta = [path_seg.theta, Traj.theta];
    Traj.v = [v_seg, Traj.v];
    Traj.phy = [phy_seg, Traj.phy];
    Traj.local_dt = [dt_seg, Traj.local_dt];

    parent_id = cur_node.parent_id;
    cur_node = grid_space_{parent_id(1), parent_id(2), parent_id(3)};
end

x = Traj.x;
y = Traj.y;
theta = Traj.theta;
v_traj = Traj.v;      % 【新增】

% ===== Step3 新增：若通过RS完成，拼接RS段到尾部 =====
if complete_via_rs_flag
    if ~isempty(rs_seg.x) && numel(rs_seg.x) > 1
        x = [x, rs_seg.x(2:end)];
        y = [y, rs_seg.y(2:end)];
        theta = [theta, rs_seg.theta(2:end)];

        % 【新增】给RS段估计v（通用做法：位移在航向方向的投影）
        dx = diff(rs_seg.x);
        dy = diff(rs_seg.y);
        th = rs_seg.theta(1:end-1);
        proj = dx .* cos(th) + dy .* sin(th);    % 位移在航向方向的投影

        v_rs = ones(1, numel(dx));
        v_rs(proj < 0) = -1;

        % 注意：拼接是从 rs_seg.x(2:end) 开始，所以 v 也对应 diff 的每一步
        v_traj = [v_traj, v_rs];
    end
    fprintf("通过RS轨迹拼接终点完成\n");
    fprintf("终点坐标 x: %f, y: %f, theta: %f\n", x(end), y(end), theta(end));
end
if completeness_flag && ~complete_via_rs_flag
    x = [x, params.task.xf];
    y = [y, params.task.yf];
    theta = [theta, params.task.thetaf];

    % 【新增】终点这个点的v，用最后一个方向延续
    if ~isempty(v_traj)
        v_traj = [v_traj, v_traj(end)];
    else
        v_traj = [1]; % 理论上不会空，保险
    end
    fprintf("通过直接拓展命中终点完成\n");
    fprintf("终点坐标 x: %f, y: %f, theta: %f\n", x(end), y(end), theta(end));
end
% ===== 基于最终路径重建完整的 phy / local_dt =====
Nx = length(x);
ldt_full = zeros(Nx,1);
phy_full = zeros(Nx,1);
for i = 1:Nx-1
    dx = x(i+1) - x(i);
    dy = y(i+1) - y(i);
    ds = hypot(dx, dy);
    ldt_full(i) = ds;
    dth = theta(i+1) - theta(i);
    while dth > pi
        dth = dth - 2*pi;
    end
    while dth < -pi
        dth = dth + 2*pi;
    end
    if ds < 1e-8
        kappa = 0;
    else
        kappa = dth / ds;
    end
    phy_full(i) = atan(kappa * params.vehicle.lw);
end
% 最后一个点补齐
if Nx >= 2
    ldt_full(Nx) = ldt_full(Nx-1);
    phy_full(Nx) = phy_full(Nx-1);
else
    ldt_full(Nx) = 0;
    phy_full(Nx) = 0;
end
%=================================================
% 可视化整条路径
if (params.ha.enable_recording)
    plot(x, y, 'k', 'LineWidth', 3);
    for ii = 1 : 10
        writeVideo(vidObj, getframe(gcf));
    end
    close(vidObj);
end

% 更新全局变量
params.ha.x = x;
params.ha.y = y;
params.ha.theta = theta;
params.ha.phy = phy_full;
params.ha.local_dt = ldt_full;
% ==================【新增】记录换向点序号 ==================
if ~exist('v_traj','var') || isempty(v_traj)
    params.change = [];
else
    v_traj(v_traj==0) = 1;
    change_idx = find(v_traj(2:end) ~= v_traj(1:end-1)) + 1;
    params.change = change_idx;
end

params.ha.v = v_traj;   % 保存生成每点方向
% ==========================================================

% ==================【新增】终端输出换向信息 ==================
n_change = numel(params.change);
fprintf('\n========== 混合A*的换向统计 ==========\n');
fprintf('总换向次数: %d\n', n_change);

if n_change == 0
    fprintf('无换向点。\n');
else
    fprintf('换向点序号及坐标（idx, x, y, dir: before->after）:\n');
    fprintf('-----------------------------------------------\n');
    for k = 1:n_change
        idx = params.change(k);

        % 保护：防止 idx 越界
        if idx < 2 || idx > numel(x)
            fprintf('#%d: idx=%d 越界，跳过\n', k, idx);
            continue;
        end

        dir_before = v_traj(idx-1);
        dir_after  = v_traj(idx);

        % 用字符更直观：F=前进, R=倒车
        if dir_before > 0, s1 = 'F'; else, s1 = 'R'; end
        if dir_after  > 0, s2 = 'F'; else, s2 = 'R'; end

        fprintf('#%d: idx=%d, x=%.3f, y=%.3f, %s->%s\n', ...
            k, idx, x(idx), y(idx), s1, s2);
    end
    fprintf('-----------------------------------------------\n');
end
fprintf('================================\n\n');
% ==========================================================

% 将theta连续化，避免跳变 > pi
for ii = 2 : length(theta)
    while (theta(ii) - theta(ii-1) > pi)
        theta(ii) = theta(ii) - 2 * pi;
    end
    while (theta(ii) - theta(ii-1) < -pi)
        theta(ii) = theta(ii) + 2 * pi;
    end
end

% 更新任务终点状态
params.task.xf = x(end);
params.task.yf = y(end);
params.task.thetaf = theta(end);


fprintf('completeness_flag = %d, complete_via_rs_flag = %d\n', ...
    completeness_flag, complete_via_rs_flag);
fprintf('回溯末端: x=%.6f, y=%.6f, theta=%.6f\n', x(end), y(end), theta(end));
fprintf('原任务终点: xf=%.6f, yf=%.6f, thetaf=%.6f\n', ...
    params.task.xf, params.task.yf, params.task.thetaf);

params.ha.success = 1;
success = 1;

end

%% --------------------辅助函数--------------------

function EstimateprogressAlongGuidingLine()
global params
s = zeros(1, length(params.guiding_path.x));
for ii = 2 : length(params.guiding_path.x)
    s(ii) = hypot(params.guiding_path.x(ii) - params.guiding_path.x(ii-1), ...
                  params.guiding_path.y(ii) - params.guiding_path.y(ii-1)) + s(ii-1); % 计算累计弧长
end
params.progress_s = fliplr(s);  % 翻转数组，从终点到起点
end

%% 启发式函数
function val = CalculateH(child_node)
global params
list = hypot(params.guiding_path.x - child_node.x, params.guiding_path.y - child_node.y); % 当前节点到引导线每点距离
ind = find(list == min(list));
ind = ind(end);  % 取最近点索引
val = params.ha.multiplier_on_heuristics * (params.progress_s(ind) + 0.1 * min(list)); % 启发式 = 剩余距离 + 微小偏差
end

%% 节点映射到离散格点索引
function idx = CalculateNodeIndex(node)
global params
params.ha_res_x = params.environment.xhorizon / params.ha.nx;  % x方向分辨率
params.ha_res_y = params.environment.yhorizon / params.ha.ny;  % y方向分辨率
params.ha_res_theta = 2 * pi / params.ha.ntheta;               % θ方向分辨率

ind1 = ceil((node.x - params.environment.xmin) / params.ha_res_x) + 1;
ind2 = ceil((node.y - params.environment.ymin) / params.ha_res_y) + 1;
ind3 = ceil(RegulateAngle(node.theta) / params.ha_res_theta) + 1;

ind1 = max(1, min(ind1, params.ha.nx));
ind2 = max(1, min(ind2, params.ha.ny));
ind3 = max(1, min(ind3, params.ha.ntheta));

idx = [ind1, ind2, ind3];
end

%% 生成扩展模式
function expansion_pattern = SpecifySamplePattern()
global params

% 在最大左转和最大右转之间，均匀取 num_phy_ha 个转角样本，作为扩展时可用的转向角集合。
phy_list = linspace(-params.vehicle.phy_max, params.vehicle.phy_max, params.ha.num_phy_ha); % 转向角候选
expansion_pattern = [];
for ii = 1 : params.ha.num_phy_ha
    expansion_pattern = [expansion_pattern;  1, phy_list(ii)];  % 动作：前进 + 当前转角
    expansion_pattern = [expansion_pattern; -1, phy_list(ii)];  % 动作：倒车 + 当前转角
end
end

%% 从openlist取最小f节点
function [cur_node, cur_node_id] = ExtractMinFNodeFromOpenlist()
global grid_space_ openlist_
list = openlist_(:, 1);
id = find(list == min(list)); id = id(end);  % 取最后一个最小f节点
cur_node_id = openlist_(id, 2 : 4);           % 节点索引
cur_node = grid_space_{cur_node_id(1), cur_node_id(2), cur_node_id(3)};
openlist_(id, :) = [];                         % 从openlist删除
grid_space_{cur_node_id(1), cur_node_id(2), cur_node_id(3)}.is_closed = 1; % 标记闭
grid_space_{cur_node_id(1), cur_node_id(2), cur_node_id(3)}.is_open = 0;
end


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

% ===== 轨迹段扫掠盒子检测 =====
% Body-only Hybrid A*: embodied-footprint sweep collision check removed.
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

%% 检查路径段合法性
function is_valid = IsPathValid(path_seg, phy, dir, local_dt)
is_valid = 0;
if (~IsVehicleBodyValid(path_seg.x, path_seg.y, path_seg.theta, phy, dir, local_dt))
    return;
end
is_valid = 1;
end

%% 可视化路径段
function PlotPathSegment(path_seg, iter)
global params
c1 = [0 255 0] / 255;   % 绿色
c2 = [255 0 0] / 255;   % 红色
d = (c2 - c1) / params.ha.iter_for_HA;
if (iter > params.ha.iter_for_HA)
    iter = params.ha.iter_for_HA;
end
col = d * iter + c1;
plot(path_seg.x, path_seg.y, 'Color', col); % 绘制路径段
plot(path_seg.x(end), path_seg.y(end), 'k.'); % 绘制末端
drawnow;
end

%%
function UpdateOpenlist(child_node, child_node_id)
% UpdateOpenlist: 删除openlist中该节点的旧条目，再插入最新f条目
% openlist_ 每行格式: [f, ix, iy, itheta]

global openlist_

if isempty(openlist_)
    openlist_ = [child_node.f, child_node_id];
    return;
end

% 找到所有匹配该id的行
mask = (openlist_(:,2) == child_node_id(1)) & ...
       (openlist_(:,3) == child_node_id(2)) & ...
       (openlist_(:,4) == child_node_id(3));

% 先删除旧条目（可能有多条）
openlist_(mask, :) = [];

% 再插入最新条目
openlist_ = [openlist_; child_node.f, child_node_id];
end

%% 【新增】
function [rs_seg, path_length] = GenerateRsPathSeg(startPose, goalPose)
global params
rs_seg = [];
path_length = Inf;

turning_radius_min = params.vehicle.lw / tan(params.vehicle.phy_max);
reedsConnObj = robotics.ReedsSheppConnection('MinTurningRadius', turning_radius_min);
% 如果你没有 turning_radius_min，就用 lw/tan(phy_max) 推一个：

% 倒车代价（可选，参考第一版）
reedsConnObj.ReverseCost = params.ha.penalty_for_backward;

[pathSegObj, ~] = connect(reedsConnObj, startPose, goalPose);
if isempty(pathSegObj) || isempty(pathSegObj{1})
    return;
end

path_length = pathSegObj{1}.Length;

% 采样步长：建议用你的ha分辨率或更小
ds = min(params.ha_res_x, params.ha_res_y);
poses = interpolate(pathSegObj{1}, 0:ds:path_length);

rs_seg.x = poses(:,1)';
rs_seg.y = poses(:,2)';
rs_seg.theta = poses(:,3)';
end

%% 【新增】(对拼接段的轨迹进行合法性检测)
function ok = IsRsPathValid(rs_seg)
global params;
ok = 0;
phy0 = 0;

if isempty(rs_seg.x) || numel(rs_seg.x) < 2
    return;
end

dx = diff(rs_seg.x);
dy = diff(rs_seg.y);
th = rs_seg.theta(1:end-1);

proj = dx .* cos(th) + dy .* sin(th);
dir_rs = ones(1, numel(proj));
dir_rs(proj < 0) = -1;

for i = 1:numel(proj)
    ds_rs = hypot(rs_seg.x(i+1)-rs_seg.x(i), rs_seg.y(i+1)-rs_seg.y(i));
    dth = rs_seg.theta(i+1) - rs_seg.theta(i);
    while dth > pi,  dth = dth - 2*pi; end
    while dth < -pi, dth = dth + 2*pi; end

    if ds_rs < 1e-8
        phy_rs = 0;
    else
        kappa_rs = dth / ds_rs;
        phy_rs = atan(kappa_rs * params.vehicle.lw);
    end

    if ~IsVehicleBodyValid(rs_seg.x(i:i+1), rs_seg.y(i:i+1), rs_seg.theta(i:i+1), phy_rs, dir_rs(i), ds_rs)
        return;
    end
end

ok = 1;
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
function DrawHAFinalMarks(cur_best_node, completeness_flag)
global params

if completeness_flag
    plot(params.task.xf, params.task.yf, 'rp', 'MarkerSize', 16, 'LineWidth', 2);
    text(params.task.xf, params.task.yf, '  Goal reached', ...
        'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
else
    plot(cur_best_node.x, cur_best_node.y, 'kp', 'MarkerSize', 16, 'LineWidth', 2);
    text(cur_best_node.x, cur_best_node.y, '  Best node on failure', ...
        'Color', 'k', 'FontSize', 12, 'FontWeight', 'bold');
end
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
