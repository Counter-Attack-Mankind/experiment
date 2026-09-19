function flag = LoadEFOptimumAndRefine()

global params

%% ====== 优化结果路径 ======
current_dir = fileparts(mfilename('fullpath'));   % Utilities
project_root = fileparts(current_dir);            % 工程根目录
base_path = fullfile(project_root, 'Results');

%% ====== 安全检查 ======
if ~exist(base_path,'dir')
    error('results 文件夹不存在，请先运行优化程序');
end
%% ====== 读取优化成功标志 ======
flag_file = fullfile(base_path,'opti_flag.txt');
if exist(flag_file,'file')
    opti_flag = load(flag_file);
else
    warning('opti_flag.txt 不存在');
    opti_flag = 0;
end

%% ====== 读取状态变量 ======
x     = readVec(base_path,'x.txt');
y     = readVec(base_path,'y.txt');
theta = readVec(base_path,'theta.txt');

v     = readVec(base_path,'v.txt');
a     = readVec(base_path,'a.txt');
phy   = readVec(base_path,'phy.txt');
w     = readVec(base_path,'w.txt');

%% ====== 读取段变量 ======
dt    = readVec(base_path,'dt.txt');
s     = readVec(base_path,'s.txt');
k     = readVec(base_path,'k.txt');

up    = readVec(base_path,'up.txt');
down  = readVec(base_path,'down.txt');
left  = readVec(base_path,'left.txt');
right = readVec(base_path,'right.txt');

%% ====== 转列向量 ======
x=x(:); y=y(:); theta=theta(:);
v=v(:); a=a(:); phy=phy(:); w=w(:);

dt=dt(:); s=s(:); k=k(:);
up=up(:); down=down(:); left=left(:); right=right(:);

%% ====== 长度检查 ======
Nstate = length(x);
Nseg   = Nstate - 1;

assert(length(dt)==Nseg,'dt 长度错误');
assert(length(s)==Nseg,'s 长度错误');
assert(length(k)==Nseg,'k 长度错误');

%% ====== theta 连续化 ======
theta_unwrap = theta;

for i = 2:Nstate

    while theta_unwrap(i) - theta_unwrap(i-1) > pi
        theta_unwrap(i) = theta_unwrap(i) - 2*pi;
    end

    while theta_unwrap(i) - theta_unwrap(i-1) < -pi
        theta_unwrap(i) = theta_unwrap(i) + 2*pi;
    end

end

%% ====== 存回 params ======
params.ef.x = x;
params.ef.y = y;
params.ef.theta = theta_unwrap;

params.ef.v = v;
params.ef.a = a;
params.ef.phy = phy;
params.ef.w = w;

params.ef.dt = dt;
params.ef.s = s;
params.ef.k = k;

params.ef.up = up;
params.ef.down = down;
params.ef.left = left;
params.ef.right = right;

params.ef.tf = sum(dt);

%% ====== 高密度插值 ======

base_step = 1e-4;

x_enriched=[];
y_enriched=[];
theta_enriched=[];

v_enriched=[];
a_enriched=[];
phy_enriched=[];
w_enriched=[];

up_enriched=[];
down_enriched=[];
left_enriched=[];
right_enriched=[];

for ii = 1:Nseg

    cur_time = dt(ii);

    nfe_seg = max(2, round(cur_time/base_step)+1);

    x_seg = linspace(x(ii),x(ii+1),nfe_seg);
    y_seg = linspace(y(ii),y(ii+1),nfe_seg);
    th_seg = linspace(theta_unwrap(ii),theta_unwrap(ii+1),nfe_seg);

    v_seg = linspace(v(ii),v(ii+1),nfe_seg);
    a_seg = linspace(a(ii),a(ii+1),nfe_seg);
    phy_seg = linspace(phy(ii),phy(ii+1),nfe_seg);
    w_seg = linspace(w(ii),w(ii+1),nfe_seg);

    up_seg    = repmat(up(ii),1,nfe_seg);
    down_seg  = repmat(down(ii),1,nfe_seg);
    left_seg  = repmat(left(ii),1,nfe_seg);
    right_seg = repmat(right(ii),1,nfe_seg);

    if ii>1

        x_seg(1)=[];
        y_seg(1)=[];
        th_seg(1)=[];

        v_seg(1)=[];
        a_seg(1)=[];
        phy_seg(1)=[];
        w_seg(1)=[];

        up_seg(1)=[];
        down_seg(1)=[];
        left_seg(1)=[];
        right_seg(1)=[];

    end

    x_enriched=[x_enriched,x_seg];
    y_enriched=[y_enriched,y_seg];
    theta_enriched=[theta_enriched,th_seg];

    v_enriched=[v_enriched,v_seg];
    a_enriched=[a_enriched,a_seg];
    phy_enriched=[phy_enriched,phy_seg];
    w_enriched=[w_enriched,w_seg];

    up_enriched=[up_enriched,up_seg];
    down_enriched=[down_enriched,down_seg];
    left_enriched=[left_enriched,left_seg];
    right_enriched=[right_enriched,right_seg];

end

%% ====== 按动画帧率重采样 ======
vis_fps   = 120;    % 视频输出帧率
vis_speed = 2.0;   % 视频播放倍速：2.0 表示 2 倍速播放
nfe_vis = max(2, round(params.ef.tf * vis_fps / vis_speed));


index = round(linspace(1,length(x_enriched),nfe_vis));
index = unique(index,'stable');

params.ef.enriched_x = x_enriched(index).';
params.ef.enriched_y = y_enriched(index).';
params.ef.enriched_theta = theta_enriched(index).';

params.ef.enriched_v = v_enriched(index).';
params.ef.enriched_a = a_enriched(index).';
params.ef.enriched_phy = phy_enriched(index).';
params.ef.enriched_w = w_enriched(index).';

params.ef.enriched_up = up_enriched(index).';
params.ef.enriched_down = down_enriched(index).';
params.ef.enriched_left = left_enriched(index).';
params.ef.enriched_right = right_enriched(index).';

%% ====== 返回标志 ======

flag = logical(opti_flag);

end


%% ====== 读取文件函数 ======

function vec = readVec(path,name)

file = fullfile(path,name);

if ~exist(file,'file')
    error('文件不存在: %s',file);
end

vec = load(file);
vec = vec(:);

end