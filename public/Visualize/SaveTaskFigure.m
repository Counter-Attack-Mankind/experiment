function SaveTaskFigure(fig_handle, image_suffix)
% ============================================================
% SaveTaskFigure
%
% 将当前任务图像保存到：
%   <scheme_dir>/Results/task_xx/
%
% 文件名格式：
%   task_xx_<image_suffix>.png
%
% 例如：
%   task_05_EFboxs.png
%   task_05_TrueBody.png
% ============================================================

global params

if nargin < 1 || isempty(fig_handle)
    fig_handle = gcf;
end

if nargin < 2 || isempty(image_suffix)
    error('SaveTaskFigure requires image_suffix.');
end

if ~isfield(params, 'io') || ...
   ~isfield(params.io, 'scheme_dir') || ...
   ~isfield(params.io, 'task_id')

    warning('SaveTaskFigure:MissingContext', ...
        'params.io.scheme_dir or params.io.task_id is missing. Figure not saved.');
    return;
end

scheme_dir = params.io.scheme_dir;
task_id    = params.io.task_id;

task_folder = fullfile( ...
    scheme_dir, ...
    'Results', ...
    sprintf('task_%02d', task_id));

if exist(task_folder, 'dir') ~= 7
    mkdir(task_folder);
end

file_name = sprintf('task_%02d_%s.png', task_id, image_suffix);
file_path = fullfile(task_folder, file_name);

% 推荐 exportgraphics，图片更稳定
exportgraphics(fig_handle, file_path, 'Resolution', 300);

fprintf('Figure saved: %s\n', file_path);

end