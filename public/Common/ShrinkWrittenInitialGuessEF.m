function ShrinkWrittenInitialGuessEF(matfile)
%SHRINKWRITTENINITIALGUESSEF
% 初始解几何可行性增强。
%
% 对发生碰撞的 buffered footprint，
% 保持车辆状态、时间网格和运动拓扑不变，
% 仅逐步缩小四个 buffering variables：
%
%   up, down, left, right
%
% 直到碰撞约束满足，或完全退化到 physical footprint。
%
% 本操作只修改 NLP initial guess，
% 不修改正式 NLP 的约束和 feasible set。

global params

if nargin < 1 || isempty(matfile)
    matfile = 'written_initial_guess_data.mat';
end

S = load(matfile);

if ~isfield(S, 'data')
    error('No variable named data found in %s.', matfile);
end

D = S.data;

scale_min  = params.ef.shrink.scale_min;
scale_step = params.ef.shrink.scale_step;

Nbox = numel(D.AX);

num_shrunk = 0;
num_unresolved = 0;


%% ========================================================================
% 逐 interval 检查
%
% NLP collision constraints 只作用于 i = 2,...,Nfe-1
% 而 Nbox = Nfe-1，因此实际检查 2:Nbox。
% ========================================================================

for i = 2:Nbox

    % 原始 EF 已经满足碰撞约束，不处理
    if intervalObstacleMinSlack(D, i) >= 0
        continue;
    end

    num_shrunk = num_shrunk + 1;

    repaired = false;

    % ------------------------------------------------------------
    % 从完整 EF 向 physical footprint 逐步收缩
    % ------------------------------------------------------------
    scale_grid = 1:-scale_step:scale_min;

    if isempty(scale_grid) || abs(scale_grid(end) - scale_min) > 1e-12
        scale_grid = [scale_grid, scale_min];
    end

    for sc = scale_grid

        C = makeScaledInterval(D, i, sc);

        if intervalObstacleMinSlack(C, i) >= 0

            D = copyIntervalEF(D, C, i);
            repaired = true;
            break;
        end

    end


    % ------------------------------------------------------------
    % 即使 physical footprint 仍然存在碰撞：
    % 不报错，不终止实验。
    %
    % 直接使用 scale=0 的完全收缩初值，
    % 后续交由 NLP 继续调整。
    % ------------------------------------------------------------
    if ~repaired

        C = makeScaledInterval(D, i, 0);

        D = copyIntervalEF(D, C, i);

        num_unresolved = num_unresolved + 1;

    end

end


%% ========================================================================
% 保存
% ========================================================================

data = D; %#ok<NASGU>
save(matfile, 'data');

rewriteIgInival(D);


fprintf('\n========== Geometric Feasibility Enhancement ==========\n');
fprintf('shrunk EF boxes      : %d\n', num_shrunk);
fprintf('unresolved at scale0 : %d\n', num_unresolved);
fprintf('========================================================\n\n');

end


%% ========================================================================
% 缩放一个 interval 的四个 buffering variables
% ========================================================================
function C = makeScaledInterval(D, i, scale)

C = D;

C.up(i)    = scale * D.up(i);
C.down(i)  = scale * D.down(i);
C.left(i)  = scale * D.left(i);
C.right(i) = scale * D.right(i);

[C.AX(i), C.AY(i), ...
 C.BX(i), C.BY(i), ...
 C.CX(i), C.CY(i), ...
 C.DX(i), C.DY(i)] = ...
    intervalBoxVertices(C, i);

end


%% ========================================================================
% 写回该 interval
% ========================================================================
function D = copyIntervalEF(D, C, i)

fields = { ...
    'up','down','left','right', ...
    'AX','AY','BX','BY', ...
    'CX','CY','DX','DY'};

for k = 1:numel(fields)

    f = fields{k};

    D.(f)(i) = C.(f)(i);

end

end


%% ========================================================================
% 根据缩放后的 buffering variables 重建 box
% ========================================================================
function [AX, AY, BX, BY, CX, CY, DX, DY] = ...
    intervalBoxVertices(D, i)

LF  = D.meta.LF;
lr  = D.meta.lr;
hlb = D.meta.hlb;

ct = cos(D.theta(i));
st = sin(D.theta(i));

AX = D.x(i) + (LF + D.up(i)) * ct ...
              - (hlb + D.left(i)) * st;

AY = D.y(i) + (LF + D.up(i)) * st ...
              + (hlb + D.left(i)) * ct;

BX = D.x(i) + (LF + D.up(i)) * ct ...
              + (hlb + D.right(i)) * st;

BY = D.y(i) + (LF + D.up(i)) * st ...
              - (hlb + D.right(i)) * ct;

CX = D.x(i) - (lr + D.down(i)) * ct ...
              + (hlb + D.right(i)) * st;

CY = D.y(i) - (lr + D.down(i)) * st ...
              - (hlb + D.right(i)) * ct;

DX = D.x(i) - (lr + D.down(i)) * ct ...
              - (hlb + D.left(i)) * st;

DY = D.y(i) - (lr + D.down(i)) * st ...
              + (hlb + D.left(i)) * ct;

end


%% ========================================================================
% 与正式 NLP collision constraints 对应的最小 slack
% ========================================================================
function min_slack = intervalObstacleMinSlack(D, i)

ef_poly = [ ...
    D.AX(i), D.AY(i);
    D.BX(i), D.BY(i);
    D.CX(i), D.CY(i);
    D.DX(i), D.DY(i)];

LF  = D.meta.LF;
lr  = D.meta.lr;
hlb = D.meta.hlb;
lb  = 2 * hlb;

% 与 NLP 中 obstacle vertex outside EF 的 RHS 一致
ef_area_rhs = ...
    (LF + lr + D.up(i) + D.down(i)) * ...
    (lb + D.right(i) + D.left(i)) + 0.1;

min_slack = inf;

for obs_idx = 1:D.meta.Nobs

    [obs_poly, obs_area_rhs] = ...
        getNlpObstacle(D.meta.obs(obs_idx));


    % obstacle vertices outside EF
    for jj = 1:size(obs_poly, 1)

        area_sum = ...
            sumPointToPolygonTriangleAreas( ...
                obs_poly(jj,:), ef_poly);

        min_slack = min( ...
            min_slack, ...
            area_sum - ef_area_rhs);

    end


    % EF vertices outside obstacle
    for jj = 1:size(ef_poly, 1)

        area_sum = ...
            sumPointToPolygonTriangleAreas( ...
                ef_poly(jj,:), obs_poly);

        min_slack = min( ...
            min_slack, ...
            area_sum - obs_area_rhs);

    end

end

end


%% ========================================================================
% 构造与 NLP 一致的 obstacle representation
% ========================================================================
function [poly, area_rhs] = getNlpObstacle(obs)

ox = obs.x(:);
oy = obs.y(:);

if numel(ox) >= 2 && ...
        abs(ox(1)-ox(end)) < 1e-12 && ...
        abs(oy(1)-oy(end)) < 1e-12

    ox(end) = [];
    oy(end) = [];

end

nv = numel(ox);

if nv == 3

    ox = [ox; ox(end)];
    oy = [oy; oy(end)];

elseif nv ~= 4

    error('Obstacle has %d vertices; only triangles/quads are supported.', nv);

end

poly = [ox, oy];

raw_area = polygonArea(poly);

area_rhs = min( ...
    raw_area + 0.02, ...
    raw_area * 1.02);

end


%% ========================================================================
% 面积工具
% ========================================================================
function area_sum = sumPointToPolygonTriangleAreas(P, poly)

area_sum = 0;
n = size(poly,1);

for k = 1:n

    p1 = poly(k,:);

    if k < n
        p2 = poly(k+1,:);
    else
        p2 = poly(1,:);
    end

    area_sum = area_sum + triangleArea(P,p1,p2);

end

end


function A = triangleArea(p1,p2,p3)

A = 0.5 * abs( ...
    (p2(1)-p1(1))*(p3(2)-p1(2)) - ...
    (p2(2)-p1(2))*(p3(1)-p1(1)));

end


function A = polygonArea(poly)

x = poly(:,1);
y = poly(:,2);

A = 0.5 * abs( ...
    sum(x .* y([2:end,1]) - ...
        y .* x([2:end,1])));

end


%% ========================================================================
% 重写 AMPL initial guess
% ========================================================================
function rewriteIgInival(D)

fid = fopen('ig.INIVAL','w');

if fid < 0
    error('Cannot open ig.INIVAL for writing.');
end

cleanup_obj = onCleanup(@() fclose(fid));

Nfe = D.meta.Nfe;

for i = 1:Nfe

    fprintf(fid,'let x[%d] := %.12f;\r\n',i,D.x(i));
    fprintf(fid,'let y[%d] := %.12f;\r\n',i,D.y(i));
    fprintf(fid,'let theta[%d] := %.12f;\r\n',i,D.theta(i));
    fprintf(fid,'let v[%d] := %.12f;\r\n',i,D.v(i));
    fprintf(fid,'let a[%d] := %.12f;\r\n',i,D.a(i));
    fprintf(fid,'let phy[%d] := %.12f;\r\n',i,D.phy(i));
    fprintf(fid,'let w[%d] := %.12f;\r\n',i,D.w(i));

end

for i = 1:Nfe-1

    fprintf(fid,'let k[%d] := %.12f;\r\n',i,D.kappa(i));
    fprintf(fid,'let dt[%d] := %.12f;\r\n',i,D.dt(i));
    fprintf(fid,'let s[%d] := %.12f;\r\n',i,D.s(i));
    fprintf(fid,'let splus[%d] := %.12f;\r\n',i,D.splus(i));
    fprintf(fid,'let sminus[%d] := %.12f;\r\n',i,D.sminus(i));

    fprintf(fid,'let up[%d] := %.12f;\r\n',i,D.up(i));
    fprintf(fid,'let down[%d] := %.12f;\r\n',i,D.down(i));
    fprintf(fid,'let left[%d] := %.12f;\r\n',i,D.left(i));
    fprintf(fid,'let right[%d] := %.12f;\r\n',i,D.right(i));

    fprintf(fid,'let AX[%d] := %.12f;\r\n',i,D.AX(i));
    fprintf(fid,'let AY[%d] := %.12f;\r\n',i,D.AY(i));
    fprintf(fid,'let BX[%d] := %.12f;\r\n',i,D.BX(i));
    fprintf(fid,'let BY[%d] := %.12f;\r\n',i,D.BY(i));
    fprintf(fid,'let CX[%d] := %.12f;\r\n',i,D.CX(i));
    fprintf(fid,'let CY[%d] := %.12f;\r\n',i,D.CY(i));
    fprintf(fid,'let DX[%d] := %.12f;\r\n',i,D.DX(i));
    fprintf(fid,'let DY[%d] := %.12f;\r\n',i,D.DY(i));

end

fprintf(fid,'let tf := %.12f;\r\n',D.tf);

end