function [x, y, theta, v, a, phy, w, time] = ...
    Scheme4ConvertPathToTraj(target_nfe)
% ============================================================
% Scheme4ConvertPathToTraj
%
% Scheme4:
%   Original Hybrid A*
%   + matched-Nfe equal-time initialization
%   + body-only NLP
%
% 与 Scheme1 的 ConvertPathToTraj 保持一致的部分：
%   1) 读取 Hybrid A* 原始路径
%   2) 删除重复点
%   3) 检测有效换向点
%   4) 按方向段调用 CalculateTimeStamp 进行时间最优速度匹配
%   5) 根据 theta / v 反求 phy
%   6) 根据 phy 计算 w
%
% 与 Scheme1 唯一的主要分叉：
%   Scheme1:
%       dense trajectory -> TimeDistribution()
%       -> EF-based nonuniform configuration-point selection
%
%   Scheme4:
%       dense trajectory -> equal-time resampling
%       -> exactly target_nfe configuration points
%
% 注意：
%   target_nfe 必须由 Scheme1 的 Nfe_config.txt 提供。
%   本函数不调用 ConvertPathToTraj，也不调用 TimeDistribution，
%   不使用任何 EF 几何筛选结果。
% ============================================================

global params

%% ============================================================
% 0. Check matched Nfe
% ============================================================

target_nfe = round(target_nfe);

if target_nfe < 2
    error('Scheme4ConvertPathToTraj: invalid target_nfe = %d.', ...
        target_nfe);
end


%% ============================================================
% 1. Read original Hybrid A* path
% ============================================================

x0 = params.ha.x;
y0 = params.ha.y;
theta0 = params.ha.theta;

if ~isfield(params.ha, 'v') || isempty(params.ha.v)
    warning('params.ha.v is missing. Assume forward motion.');
    v0 = ones(size(x0));
else
    v0 = params.ha.v;
end


%% ============================================================
% 2. Remove repeated / stationary points
% ============================================================

xx = x0(1);
yy = y0(1);
tt = theta0(1);
vv = v0(1);

for ii = 2:length(x0)

    if ~((x0(ii) == x0(ii-1)) && ...
         (y0(ii) == y0(ii-1)))

        xx = [xx, x0(ii)]; %#ok<AGROW>
        yy = [yy, y0(ii)]; %#ok<AGROW>
        tt = [tt, theta0(ii)]; %#ok<AGROW>
        vv = [vv, v0(ii)]; %#ok<AGROW>
    end
end

x0 = xx;
y0 = yy;
theta0 = tt;
v0 = vv;

if numel(x0) < 2
    error('Scheme4ConvertPathToTraj: Hybrid A* path has too few points.');
end

num_samples = numel(x0);


%% ============================================================
% 3. Detect effective direction switches
%    Keep exactly the same logic as Scheme1
% ============================================================

min_length = 2;

[change_idx, seg_st, seg_ed, seg_dir, seg_len] = ...
    getEffectiveChangeIdx(x0, y0, v0, min_length);

fprintf('\n========== Scheme4 Direction Segments ==========\n');
fprintf('Effective direction switches: %d\n', numel(change_idx));

for k = 1:length(seg_st)

    if seg_dir(k) > 0
        dir_str = 'forward';
    else
        dir_str = 'reverse';
    end

    fprintf('Segment #%d: [%d -> %d], %s, length=%.3f m\n', ...
        k, seg_st(k), seg_ed(k), dir_str, seg_len(k));
end

for k = 1:numel(change_idx)
    idx = change_idx(k);

    fprintf('Switch #%d: idx=%d, x=%.3f, y=%.3f\n', ...
        k, idx, x0(idx), y0(idx));
end


%% ============================================================
% 4. Unwrap theta
% ============================================================

for ii = 2:num_samples

    while theta0(ii) - theta0(ii-1) > pi
        theta0(ii) = theta0(ii) - 2*pi;
    end

    while theta0(ii) - theta0(ii-1) < -pi
        theta0(ii) = theta0(ii) + 2*pi;
    end
end


%% ============================================================
% 5. Time-optimal velocity matching
%    Same construction as Scheme1
% ============================================================

if isempty(change_idx)

    % --------------------------------------------------------
    % Single direction
    % --------------------------------------------------------

    [terminal_time, x_dense, y_dense, theta_dense, ...
        v_mag, a_mag] = ...
        CalculateTimeStamp(x0, y0, theta0);

    sgn = sign(v0(1));

    if sgn == 0
        sgn = 1;
    end

    v_dense = abs(v_mag) * sgn;
    a_dense = a_mag * sgn;

else

    % --------------------------------------------------------
    % Multiple direction segments
    % --------------------------------------------------------

    N = numel(x0);
    cut_idx = [1, change_idx, N];

    x_dense = [];
    y_dense = [];
    theta_dense = [];

    v_dense = [];
    a_dense = [];

    terminal_time = 0;

    nSeg = length(cut_idx) - 1;

    for s = 1:nSeg

        st = cut_idx(s);
        ed = cut_idx(s+1);

        if ed <= st
            continue;
        end

        x_seg = x0(st:ed);
        y_seg = y0(st:ed);
        theta_seg = theta0(st:ed);

        [Ts, x_seg_dense, y_seg_dense, theta_seg_dense, ...
            v_mag_seg, a_mag_seg] = ...
            CalculateTimeStamp(x_seg, y_seg, theta_seg);

        % Direction of this segment
        sgn = sign(v0(st));

        if sgn == 0

            if st > 1
                sgn = sign(v0(st-1));
            end

            if sgn == 0
                sgn = 1;
            end
        end

        v_seg = abs(v_mag_seg) * sgn;
        a_seg = a_mag_seg * sgn;

        % Same treatment as Scheme1:
        % direction-switch boundary velocity = 0
        if s > 1
            v_seg(1) = 0;
        end

        if s < nSeg
            v_seg(end) = 0;
        end

        if ~isempty(a_seg)

            if s > 1
                a_seg(1) = 0;
            end

            if s < nSeg
                a_seg(end) = 0;
            end
        end

        % Concatenate.
        % Remove the first point of subsequent segments to avoid
        % repeated geometry at the switching point.
        if isempty(x_dense)

            x_dense = x_seg_dense;
            y_dense = y_seg_dense;
            theta_dense = theta_seg_dense;

            v_dense = v_seg;
            a_dense = a_seg;

        else

            x_dense = [x_dense, x_seg_dense(2:end)]; %#ok<AGROW>
            y_dense = [y_dense, y_seg_dense(2:end)]; %#ok<AGROW>
            theta_dense = [theta_dense, ...
                theta_seg_dense(2:end)]; %#ok<AGROW>

            v_dense = [v_dense, v_seg(2:end)]; %#ok<AGROW>
            a_dense = [a_dense, a_seg(2:end)]; %#ok<AGROW>
        end

        terminal_time = terminal_time + Ts;
    end
end


%% ============================================================
% 6. Calculate phy and w on the dense time trajectory
%    Same basic treatment as Scheme1
% ============================================================

dense_n = numel(x_dense);

if dense_n < 2
    error('Scheme4ConvertPathToTraj: dense trajectory has too few points.');
end

dense_dt = terminal_time / (dense_n - 1);

phy_dense = zeros(1, dense_n);
w_dense = zeros(1, dense_n);

% Steering angle
for ii = 2:(dense_n - 1)

    if abs(v_dense(ii)) < 1e-6

        phy_dense(ii) = 0;

    else

        phy_dense(ii) = atan( ...
            (theta_dense(ii+1) - theta_dense(ii)) ...
            * params.vehicle.lw ...
            / (dense_dt * v_dense(ii)));
    end

    phy_dense(ii) = min( ...
        max(phy_dense(ii), -params.vehicle.phy_max), ...
        params.vehicle.phy_max);
end

phy_dense(isnan(phy_dense)) = 0;


% Steering rate
for ii = 2:(dense_n - 1)

    w_dense(ii) = ...
        (phy_dense(ii+1) - phy_dense(ii)) / dense_dt;

    w_dense(ii) = min( ...
        max(w_dense(ii), -params.vehicle.w_max), ...
        params.vehicle.w_max);
end


%% ============================================================
% 7. Equal-time matched-Nfe resampling
%
% This replaces Scheme1's TimeDistribution().
% ============================================================

t_dense = linspace(0, terminal_time, dense_n);
t_plan = linspace(0, terminal_time, target_nfe);

x = interp1(t_dense, x_dense, t_plan, 'linear');
y = interp1(t_dense, y_dense, t_plan, 'linear');
theta = interp1(t_dense, theta_dense, t_plan, 'linear');

v = interp1(t_dense, v_dense, t_plan, 'linear');
a = interp1(t_dense, a_dense, t_plan, 'linear');

phy = interp1(t_dense, phy_dense, t_plan, 'linear');
w = interp1(t_dense, w_dense, t_plan, 'linear');


%% ============================================================
% 8. Equal dt
% ============================================================

dt = terminal_time / (target_nfe - 1);

time = [ ...
    dt * ones(1, target_nfe - 1), ...
    0 ...
    ];


%% ============================================================
% 9. Enforce endpoint values consistent with NLP4
% ============================================================

v(1) = 0;
v(end) = 0;

a(1) = 0;
a(end) = 0;

phy(1) = 0;
phy(end) = 0;

w(1) = 0;
w(end) = 0;


%% ============================================================
% 10. Store basic information
% ============================================================

params.nfe = target_nfe;

params.scheme4.target_nfe = target_nfe;
params.scheme4.terminal_time = terminal_time;
params.scheme4.equal_dt = dt;
params.scheme4.source_dense_count = dense_n;


%% ============================================================
% 11. Output
% ============================================================

fprintf('\n========== Scheme4 Trajectory Conversion ==========\n');
fprintf('Matched Nfe                 : %d\n', target_nfe);
fprintf('Dense trajectory count      : %d\n', dense_n);
fprintf('Initial terminal time       : %.6f s\n', terminal_time);
fprintf('Equal dt                    : %.6f s\n', dt);
fprintf('dt range                    : %.6f / %.6f s\n', ...
    min(time(1:end-1)), max(time(1:end-1)));
fprintf('No TimeDistribution was used.\n');
fprintf('No EF-based point selection was used.\n');
fprintf('====================================================\n\n');

end