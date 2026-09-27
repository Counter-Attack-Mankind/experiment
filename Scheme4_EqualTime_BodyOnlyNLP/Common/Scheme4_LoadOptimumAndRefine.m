function flag = Scheme4_LoadOptimumAndRefine()
% Load plan1 optimized variables into params.ef for existing visualization.
global params

flag = false;
if exist(fullfile('results', 'opti_flag.txt'), 'file') ~= 2
    return;
end
opti_flag = load(fullfile('results', 'opti_flag.txt'));
if isempty(opti_flag) || opti_flag(1) == 0
    return;
end

names = {'x','y','theta','v','a','phy','w','dt'};
for i = 1:numel(names)
    file = fullfile('results', [names{i}, '.txt']);
    if exist(file, 'file') ~= 2
        return;
    end
    S.(names{i}) = load(file); %#ok<AGROW>
end

params.ef.x = S.x(:);
params.ef.y = S.y(:);
params.ef.theta = S.theta(:);
params.ef.v = S.v(:);
params.ef.a = S.a(:);
params.ef.phy = S.phy(:);
params.ef.w = S.w(:);
params.ef.dt = S.dt(:);
params.ef.time = [S.dt(:); 0];

[params.ef.enriched_x, params.ef.enriched_y, ...
 params.ef.enriched_theta, params.ef.enriched_v] = enrichPose( ...
    params.ef.x, params.ef.y, params.ef.theta, params.ef.v);

params.nlp.nfe = numel(params.ef.x);
flag = true;
end

function [xe, ye, the, ve] = enrichPose(x, y, theta, v)
xe = [];
ye = [];
the = [];
ve = [];
for i = 1:numel(x)-1
    nseg = max(2, ceil(hypot(x(i+1)-x(i), y(i+1)-y(i)) / 0.02));
    t = linspace(0, 1, nseg);
    if i > 1
        t = t(2:end);
    end
    xe = [xe; x(i) + (x(i+1)-x(i)) * t(:)]; %#ok<AGROW>
    ye = [ye; y(i) + (y(i+1)-y(i)) * t(:)]; %#ok<AGROW>
    the = [the; theta(i) + (theta(i+1)-theta(i)) * t(:)]; %#ok<AGROW>
    ve = [ve; repmat(v(i), numel(t), 1)]; %#ok<AGROW>
end
end
