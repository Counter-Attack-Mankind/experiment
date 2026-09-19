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
end
end

function copyIfExists(src, dst)
if exist(src, 'file') == 2
    copyfile(src, dst);
end
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
