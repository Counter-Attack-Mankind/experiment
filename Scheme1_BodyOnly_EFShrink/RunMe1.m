%% ==== 加载工作区间与对应文件 ==========
% 本文件是（原版混合A*+初始解几何可行性增强+时间可行性增强+缓冲避障模型LSE光滑化），论文中采用的思路

clear all; close all; clc;
scheme_dir = fileparts(mfilename('fullpath'));
experiment_root = fileparts(scheme_dir);
public_dir = fullfile(experiment_root, 'public');

% =========================
% 公共代码
% =========================
addpath(fullfile(public_dir, 'Utilities'));
addpath(fullfile(public_dir, 'Common'));
addpath(fullfile(public_dir, 'Environment'));
addpath(fullfile(public_dir, 'Visualize'));
addpath(fullfile(public_dir, 'check'));

% =========================
% Scheme1 专用代码
% =========================
addpath(scheme_dir);
addpath(fullfile(scheme_dir, 'Common'));
addpath(fullfile(scheme_dir, 'HybridA'));
addpath(fullfile(scheme_dir, 'ConvertTraj'));

% 所有运行时文件都在 Scheme1 目录生成
cd(scheme_dir);
fprintf('\n===== Active Scheme Functions =====\n');
fprintf('ConvertPathToTraj        : %s\n', which('ConvertPathToTraj'));
fprintf('TimeDistribution         : %s\n', which('TimeDistribution'));
fprintf('SearchTrajViaHybridAstar : %s\n', which('SearchTrajViaHybridAstar'));
fprintf('===================================\n');
%% ===== 基础初始化 =====

global params

task_id = 3;
params.task_id = task_id;

run_paths = PrepareStrategyRunFolders(scheme_dir, task_id);

InitializeParams();
LoadTask(task_id);

%% ==== Hybrid A* ====

params.ha.enable_debug_plot = 1;
params.ha.debug_plot_stride = 50;
params.ha.strategy_name = 'Scheme1_BodyOnly_EFShrink';

params.ha.sweep_scale = 0;
params.visualize.show_ef_boxes = 1;

fprintf('\n========== Scheme 1: Body-only Hybrid A* + EF shrink repair + NLP ==========\n');

success = SearchTrajViaHybridAstar();

if ~success
    error('Scheme 1 Hybrid A* failed: %s', params.ha.fail_reason);
else
    VisualizeHybridAstarPath();
end

%% ==== Add velocity and configuration-point selection ====

params.ef.max_dt = 0.25;
params.ef.config_shrink_scale = 0.9;

[x, y, theta, v, a, phy, w, time] = ConvertPathToTraj();

VisualizeEmbodimentFilteredTraj(x, y);

scheme1_ef_report = CheckInitialEFCollisionForScheme1( ...
    x, y, theta, v, phy, time(1:end-1));

save( ...
    fullfile(run_paths.initial_guess, ...
    sprintf('scheme1_initial_ef_report_task_%02d.mat', task_id)), ...
    'scheme1_ef_report');

%% ==== EF shrink ====

params.scheme1.shrink.scale_min = 0;
params.scheme1.shrink.scale_step = 0.02;
params.scheme1.shrink.safety_slack = 1e-7;

%% ==== Initial guess write and check ====

WriteEFInitialGuess( ...
    x, y, theta, v, a, phy, w, time(1:end-1));

scheme1_ef_shrink_report = ...
    ShrinkWrittenInitialGuessEFForScheme1( ...
    'written_initial_guess_data.mat');

save( ...
    fullfile(run_paths.initial_guess, ...
    sprintf('scheme1_shrunk_ef_report_task_%02d.mat', task_id)), ...
    'scheme1_ef_shrink_report');

report = CheckWrittenInitialGuessForNLP();

ArchiveStrategyRunFiles(scheme_dir, task_id, report, 'initial');

%% ==== IPOPT / AMPL ====

tic
solver_dir = fullfile(public_dir, 'solver');
ampl_log_file = fullfile(run_paths.root, 'ampl_log.txt');

ampl_exe = fullfile(public_dir, 'solver', 'ampl.exe');   %指出对应的路径
rr_file = fullfile(scheme_dir, 'rr1.run');   % 检测rr.run与NLP.mod是否存在，并且读取路径
nlp_file = fullfile(scheme_dir, 'NLP1.mod');

%日志调试
fprintf('\nRunning Scheme 1 AMPL solver\n');
fprintf('AMPL executable: %s\n', ampl_exe);
fprintf('AMPL driver    : %s\n', rr_file);
fprintf('NLP model      : %s\n', nlp_file);
fprintf('AMPL log file  : %s\n', ampl_log_file);

% 当前工作目录已经是 Scheme1
cmd = sprintf('"%s" rr1.run', ampl_exe);
[ampl_status, ampl_output] = system(cmd);

solve_time = toc;

fprintf('%s\n', ampl_output);
fprintf('AMPL elapsed time: %.6f s\n', solve_time);

fid = fopen(ampl_log_file, 'w');

if fid >= 0
    fprintf(fid, '%s', ampl_output);
    fclose(fid);
end

ArchiveStrategyRunFiles(scheme_dir, task_id, report, 'all');

%% ==== Success plot ====

flag = LoadScheme1EFOptimumAndRefine(scheme_dir, task_id);

if flag
    PlotEFBoxesAndTrueSweptArea();
    PlotTrueVehicleSweptAreaOnly();
else
    fprintf('Scheme 1 optimization failed.\n');
end