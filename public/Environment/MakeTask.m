% clear all; close all; clc;
addpath('Utilities');
addpath('Common');
addpath('CaseData');
addpath('Data_test');
addpath('HybridA');
addpath('ConvertTraj');
addpath('Environment\')

%% ===== 任务配置与初始化 =====
task_id = 15;   % 只改这里
task_file = fullfile('D:\desktop\具身足迹毕设\embodyfootprint-hybrid\Environment\Data_test', [num2str(task_id), '.mat']);
InitializeParams();
%MakeTaskByMouse(task_file);
%EditTaskObstacles(task_file, task_file);
%modify_target(task_id);

DrawEnvironment(task_file);