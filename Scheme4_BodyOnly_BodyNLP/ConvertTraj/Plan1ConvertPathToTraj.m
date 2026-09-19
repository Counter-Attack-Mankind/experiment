function [x, y, theta, v, a, phy, w, time, target_nfe] = Plan1ConvertPathToTraj()
% Plan 1 initial trajectory:
% 1) run the copied configuration filtering only to obtain its final Nfe;
% 2) resample the original dense Hybrid A* path to that Nfe at equal index
%    intervals; no embodied-footprint variables are used downstream.

global params

ha_x = params.ha.x(:);
ha_y = params.ha.y(:);
ha_theta = unwrap(params.ha.theta(:));
if isfield(params.ha, 'v') && ~isempty(params.ha.v)
    ha_dir = params.ha.v(:);
else
    ha_dir = ones(size(ha_x));
end

[~, ~, ~, ~, ~, ~, ~, screened_time] = ConvertPathToTraj();
target_nfe = params.nfe;
total_time = sum(screened_time(1:end-1));

if target_nfe < 2
    error('Plan1ConvertPathToTraj: invalid target Nfe = %d.', target_nfe);
end
if total_time <= 0
    total_time = max(target_nfe - 1, 1) * params.ef.max_dt;
end

keep = [true; hypot(diff(ha_x), diff(ha_y)) > 1e-10];
ha_x = ha_x(keep);
ha_y = ha_y(keep);
ha_theta = ha_theta(keep);
ha_dir = ha_dir(keep);

n_dense = numel(ha_x);
if n_dense < 2
    error('Plan1ConvertPathToTraj: Hybrid A* path has too few points.');
end

q_dense = 1:n_dense;
q_plan = linspace(1, n_dense, target_nfe);

x = interp1(q_dense, ha_x, q_plan, 'linear').';
y = interp1(q_dense, ha_y, q_plan, 'linear').';
theta = interp1(q_dense, ha_theta, q_plan, 'linear').';
dir_idx = min(max(round(q_plan), 1), n_dense);
dir_sign = sign(ha_dir(dir_idx)).';
dir_sign(dir_sign == 0) = 1;

dt = total_time / (target_nfe - 1);
time = [dt * ones(target_nfe - 1, 1); 0];

v = zeros(target_nfe, 1);
a = zeros(target_nfe, 1);
phy = zeros(target_nfe, 1);
w = zeros(target_nfe, 1);

for i = 1:target_nfe-1
    ds = hypot(x(i+1) - x(i), y(i+1) - y(i));
    v(i) = dir_sign(i) * min(ds / dt, params.vehicle.v_max);

    dtheta = theta(i+1) - theta(i);
    if abs(v(i)) > 1e-8
        phy(i) = atan(dtheta * params.vehicle.lw / (dt * v(i)));
        phy(i) = min(max(phy(i), -params.vehicle.phy_max), params.vehicle.phy_max);
    end
end

v(end) = 0;
phy(end) = 0;
v(1) = 0;
phy(1) = 0;

for i = 1:target_nfe-1
    a(i) = (v(i+1) - v(i)) / dt;
    w(i) = (phy(i+1) - phy(i)) / dt;
end
a = min(max(a, -params.vehicle.a_max), params.vehicle.a_max);
w = min(max(w, -params.vehicle.w_max), params.vehicle.w_max);
a([1 end]) = 0;
w([1 end]) = 0;

params.nfe = target_nfe;
params.plan1.target_nfe = target_nfe;
params.plan1.total_time = total_time;
params.plan1.source_dense_count = n_dense;

fprintf('\n========== Plan 1 Trajectory Conversion ==========\n');
fprintf('screened Nfe used only as count : %d\n', target_nfe);
fprintf('dense HA path count             : %d\n', n_dense);
fprintf('equal dt                        : %.6f\n', dt);
fprintf('total time                      : %.6f\n', total_time);
fprintf('==================================================\n\n');
end
