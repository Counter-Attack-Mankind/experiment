function ArchiveStrategyRunFiles(strategy_dir, task_id, report, phase)
if nargin < 4
    phase = 'all';
end
paths = PrepareStrategyRunFolders(strategy_dir, task_id);
project_root = fileparts(fileparts(strategy_dir));

copyIfExists(fullfile(project_root, 'ig.INIVAL'), paths.initial_guess);
copyIfExists(fullfile(project_root, 'PV'), paths.initial_guess);
copyIfExists(fullfile(project_root, 'PPP'), paths.initial_guess);
copyIfExists(fullfile(project_root, 'Area'), paths.initial_guess);
copyIfExists(fullfile(project_root, 'written_initial_guess_data.mat'), paths.initial_guess);

if nargin >= 3
    save(fullfile(paths.initial_guess, sprintf('initial_guess_report_task_%02d.mat', task_id)), 'report');
end

result_dir = fullfile(project_root, 'results');
if isfolder(result_dir) && any(strcmpi(phase, {'all','optimized'}))
    files = dir(fullfile(result_dir, '*.txt'));
    for i = 1:numel(files)
        copyfile(fullfile(files(i).folder, files(i).name), paths.optimized_variables);
    end
    writeCombinedOptimizedVariables(paths.optimized_variables, task_id);
    writeOptimizedVariablesCsv(strategy_dir, result_dir, task_id);
end
end

function copyIfExists(src, dst)
if exist(src, 'file') == 2
    copyfile(src, dst);
end
end

function writeOptimizedVariablesCsv(strategy_dir, source_dir, task_id)
target_dir = fullfile(strategy_dir, 'result', sprintf('task_%02d', task_id));
if ~isfolder(target_dir)
    mkdir(target_dir);
end

names = {'x','y','theta','v','a','phy','w','dt','s','splus','sminus','k','up','down','left','right'};
values = struct();
max_len = 0;
for i = 1:numel(names)
    file = fullfile(source_dir, [names{i}, '.txt']);
    if exist(file, 'file') == 2
        values.(names{i}) = load(file);
        values.(names{i}) = values.(names{i})(:);
        max_len = max(max_len, numel(values.(names{i})));
    else
        values.(names{i}) = [];
    end
end

if max_len == 0
    warning('No optimized variable txt files found in: %s', source_dir);
    return;
end

T = table((1:max_len)', 'VariableNames', {'row'});
for i = 1:numel(names)
    col = nan(max_len, 1);
    v = values.(names{i});
    if ~isempty(v)
        col(1:numel(v)) = v;
    end
    T.(names{i}) = col;
end

flag_file = fullfile(source_dir, 'opti_flag.txt');
if exist(flag_file, 'file') == 2
    opti_flag = load(flag_file);
else
    opti_flag = nan;
end

meta_file = fullfile(target_dir, 'meta.csv');
meta = table(task_id, opti_flag, 'VariableNames', {'task_id', 'opti_flag'});
writetable(meta, meta_file);

csv_file = fullfile(target_dir, sprintf('optimized_variables_task_%02d.csv', task_id));
writetable(T, csv_file);
fprintf('Scheme 2 CSV result saved: %s\n', csv_file);
end

function writeCombinedOptimizedVariables(result_dir, task_id)
names = {'x','y','theta','v','a','phy','w','dt','s','splus','sminus','k','up','down','left','right'};
out_file = fullfile(result_dir, sprintf('optimized_variables_task_%02d.txt', task_id));
fid = fopen(out_file, 'w');
if fid < 0
    error('Cannot create file: %s', out_file);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, 'task_id %d\r\n', task_id);
for i = 1:numel(names)
    file = fullfile(result_dir, [names{i}, '.txt']);
    if exist(file, 'file') ~= 2
        continue;
    end
    values = load(file);
    fprintf(fid, '\r\n[%s]\r\n', names{i});
    for j = 1:numel(values)
        fprintf(fid, '%d %.12f\r\n', j, values(j));
    end
end
end
