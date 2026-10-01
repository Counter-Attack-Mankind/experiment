%% ==== 加载工作区间与对应文件 ==========
% 本文件是（原版混合A*+真实车身三角面积避障约束）

clear all; close all; clc;
scheme_dir = fileparts(mfilename('fullpath'));  %获取当前运行的RunMe.m的完整路径 
experiment_root = fileparts(scheme_dir);        %向上回退得到总实验路径 experiment/
public_dir = fullfile(experiment_root, 'public');   %公共模块放在 experiment/public下

addpath(fullfile(public_dir, 'Utilities'));
addpath(fullfile(public_dir, 'Common'));
addpath(fullfile(public_dir, 'Environment'));
addpath(fullfile(public_dir, 'Visualize'));
addpath(fullfile(public_dir, 'check'));
addpath(fullfile(public_dir, 'hybridAstar'));
addpath(fullfile(public_dir, 'ConvertTraj'));
addpath(scheme_dir);
addpath(fullfile(scheme_dir, 'Common'));

% 所有运行时文件都在 Scheme4 目录生成
cd(scheme_dir);

fprintf('\n===== Active Scheme Functions =====\n');
fprintf('ConvertPathToTraj        : %s\n', which('ConvertPathToTraj'));
fprintf('TimeDistribution         : %s\n', which('TimeDistribution'));
fprintf('SearchTrajViaHybridAstar : %s\n', which('SearchTrajViaHybridAstar'));
fprintf('===================================\n');

%% ===== 基础初始化 =====
global params
task_id = 21;
params.task_id = task_id;
run_paths = PrepareStrategyRunFolders(scheme_dir, task_id);
InitializeParams();
LoadTask(task_id);
params.io.scheme_dir = scheme_dir;
params.io.task_id    = task_id;

%% ==== hybrid A=======
params.ha.enable_debug_plot = 0;
params.ha.debug_plot_stride = 50;
params.ha.strategy_name = 'Scheme4_body_only_baseline';
params.ha.sweep_scale = 0;

fprintf('\n========== Scheme4: body-only Hybrid A* + Nfe count + body-only NLP ==========\n');
success = SearchTrajViaHybridAstar();
if ~success
    error('Scheme4 Hybrid A* failed: %s', params.ha.fail_reason);
end

%% ===== add velocity and choose point =====

target_nfe = ReadMatchedNfe(task_id, experiment_root);      %从scheme1中读取Nfe
[x, y, theta, v, a, phy, w, time] = Scheme4ConvertPathToTraj(target_nfe);
fprintf('Scheme4 final Nfe count: %d\n', target_nfe);
Scheme4_WriteInitialGuess(x, y, theta, v, a, phy, w, time(1:end-1));
%VisualizeEmbodimentFilteredTraj(x, y);
Scheme4_ArchiveRunFiles(scheme_dir, task_id, 'initial');

%% ==== IPOPT / AMPL ====

solver_dir = fullfile(public_dir, 'solver');
ampl_log_file = fullfile(run_paths.root, 'ampl_log.txt');
ampl_exe = fullfile(public_dir, 'solver', 'ampl.exe');   %指出对应的路径
rr_file = fullfile(scheme_dir, 'rr4.run');   % 检测rr.run与NLP.mod是否存在，并且读取路径
nlp_file = fullfile(scheme_dir, 'NLP4.mod');

%日志调试
fprintf('\nRunning Scheme 4 AMPL solver\n');
fprintf('AMPL executable: %s\n', ampl_exe);
fprintf('AMPL driver    : %s\n', rr_file);
fprintf('NLP model      : %s\n', nlp_file);
fprintf('AMPL log file  : %s\n', ampl_log_file);

% 当前工作目录已经是 Scheme4
cmd = sprintf('"%s" rr4.run', ampl_exe);
[ampl_status, ampl_output] = system(cmd);

fprintf('%s\n', ampl_output);
fid = fopen(ampl_log_file, 'w');

if fid >= 0
    fprintf(fid, '%s', ampl_output);
    fclose(fid);
end

%% ==== Archive optimized results ======
Scheme4_ArchiveRunFiles(scheme_dir, task_id, 'optimized');


%% ==== Unified evaluation and plot ====
evaluation_result = EvaluateOptimizationResult(scheme_dir, task_id);
if evaluation_result.success
    flag = Scheme4_LoadOptimumAndRefine();
    if flag
        %PlotTrueVehicleSweptAreaOnly();
        %Final_Viusalize_withplot();
    else
        fprintf('Scheme 4 optimization failed.\n');
    end
end