function VisualizeInitialEFCollision(matfile)
%VISUALIZEINITIALEFCOLLISION
% 可视化真正写入 NLP 的初始 buffered footprints。
%
% 蓝色：满足 NLP obstacle collision constraints
% 红色：违反 NLP obstacle collision constraints
%
% 本函数直接读取 written_initial_guess_data.mat，
% 不重新计算 EF，因此显示的就是实际送入 NLP 的 initial guess。

global params

if nargin < 1 || isempty(matfile)
    matfile = 'written_initial_guess_data.mat';
end

S = load(matfile);

if ~isfield(S,'data')
    error('No variable named data found in %s.',matfile);
end

D = S.data;

Nbox = numel(D.AX);

collision = false(Nbox,1);

% 正式 NLP obstacle constraints 为 i=2,...,Nfe-1
for i = 2:Nbox

    collision(i) = ...
        intervalObstacleMinSlack(D,i) < 0;

end


%% ========================================================================
% 绘图
% ========================================================================

figure( ...
    'Name','Initial EF Collision Check', ...
    'Color','w');

hold on;
axis equal;
box on;
grid on;

xlabel('x / m');
ylabel('y / m');

axis([ ...
    params.environment.xmin, ...
    params.environment.xmax, ...
    params.environment.ymin, ...
    params.environment.ymax]);


% ------------------------------------------------------------------------
% 障碍物
% ------------------------------------------------------------------------
for i = 1:numel(params.environment.obs)

    patch( ...
        params.environment.obs(i).x(:), ...
        params.environment.obs(i).y(:), ...
        [0.75 0.75 0.75], ...
        'EdgeColor',[0.2 0.2 0.2], ...
        'FaceAlpha',0.8, ...
        'HandleVisibility','off');

end


% ------------------------------------------------------------------------
% 参考轨迹
% ------------------------------------------------------------------------
plot( ...
    D.x, D.y, ...
    'k--', ...
    'LineWidth',1.0, ...
    'DisplayName','Initial trajectory');


% ------------------------------------------------------------------------
% EF boxes
% ------------------------------------------------------------------------
for i = 1:Nbox

    X = [ ...
        D.AX(i), ...
        D.BX(i), ...
        D.CX(i), ...
        D.DX(i), ...
        D.AX(i)];

    Y = [ ...
        D.AY(i), ...
        D.BY(i), ...
        D.CY(i), ...
        D.DY(i), ...
        D.AY(i)];

    if collision(i)

        patch( ...
            X,Y, ...
            [1.0 0.65 0.65], ...
            'FaceAlpha',0.35, ...
            'EdgeColor',[0.85 0.05 0.05], ...
            'LineWidth',2.0, ...
            'HandleVisibility','off');

        text( ...
            mean(X(1:4)), ...
            mean(Y(1:4)), ...
            sprintf('%d',i), ...
            'Color',[0.85 0.05 0.05], ...
            'FontWeight','bold', ...
            'HorizontalAlignment','center');

    else

        patch( ...
            X,Y, ...
            [0.70 0.84 1.0], ...
            'FaceAlpha',0.10, ...
            'EdgeColor',[0.10 0.35 0.85], ...
            'LineWidth',0.8, ...
            'HandleVisibility','off');

    end

end


% legend proxy
h1 = plot(nan,nan,'-', ...
    'Color',[0.10 0.35 0.85], ...
    'LineWidth',1.5, ...
    'DisplayName','Collision-free EF');

h2 = plot(nan,nan,'-', ...
    'Color',[0.85 0.05 0.05], ...
    'LineWidth',2.0, ...
    'DisplayName','Colliding EF');

legend([h1,h2],'Location','bestoutside');


bad_idx = find(collision);

title(sprintf( ...
    'Initial Buffered Footprints: %d colliding / %d checked', ...
    numel(bad_idx), ...
    max(Nbox-1,0)));

fprintf('\n========== Initial EF Collision Visualization ==========\n');
fprintf('checked EF boxes : %d\n',max(Nbox-1,0));
fprintf('colliding boxes  : %d\n',numel(bad_idx));

if ~isempty(bad_idx)
    fprintf('indices          : ');
    fprintf('%d ',bad_idx);
    fprintf('\n');
end

fprintf('========================================================\n\n');

end


%% ========================================================================
% Collision slack
% ========================================================================
function min_slack = intervalObstacleMinSlack(D,i)

ef_poly = [ ...
    D.AX(i),D.AY(i);
    D.BX(i),D.BY(i);
    D.CX(i),D.CY(i);
    D.DX(i),D.DY(i)];

LF  = D.meta.LF;
lr  = D.meta.lr;
hlb = D.meta.hlb;
lb  = 2*hlb;

ef_area_rhs = ...
    (LF + lr + D.up(i) + D.down(i)) * ...
    (lb + D.right(i) + D.left(i)) + 0.1;

min_slack = inf;

for obs_idx = 1:D.meta.Nobs

    [obs_poly,obs_area_rhs] = ...
        getNlpObstacle(D.meta.obs(obs_idx));

    for j = 1:size(obs_poly,1)

        area_sum = ...
            pointPolygonAreaSum( ...
                obs_poly(j,:), ...
                ef_poly);

        min_slack = min( ...
            min_slack, ...
            area_sum-ef_area_rhs);

    end

    for j = 1:4

        area_sum = ...
            pointPolygonAreaSum( ...
                ef_poly(j,:), ...
                obs_poly);

        min_slack = min( ...
            min_slack, ...
            area_sum-obs_area_rhs);

    end

end

end


function [poly,area_rhs] = getNlpObstacle(obs)

ox = obs.x(:);
oy = obs.y(:);

if numel(ox)>=2 && ...
        abs(ox(1)-ox(end))<1e-12 && ...
        abs(oy(1)-oy(end))<1e-12

    ox(end)=[];
    oy(end)=[];

end

if numel(ox)==3

    ox=[ox;ox(end)];
    oy=[oy;oy(end)];

elseif numel(ox)~=4

    error('Only triangle/quad obstacles are supported.');

end

poly=[ox,oy];

area=polygonArea(poly);

area_rhs=min(area+0.02,area*1.02);

end


function s=pointPolygonAreaSum(P,poly)

s=0;

for i=1:size(poly,1)

    j=i+1;

    if j>size(poly,1)
        j=1;
    end

    s=s+triangleArea(P,poly(i,:),poly(j,:));

end

end


function A=triangleArea(p1,p2,p3)

A=0.5*abs( ...
    (p2(1)-p1(1))*(p3(2)-p1(2)) - ...
    (p2(2)-p1(2))*(p3(1)-p1(1)));

end


function A=polygonArea(poly)

x=poly(:,1);
y=poly(:,2);

A=0.5*abs( ...
    sum(x.*y([2:end,1])- ...
        y.*x([2:end,1])));

end