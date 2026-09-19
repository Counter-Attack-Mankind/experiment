function ArchivePlan1RunFiles(plan_dir, task_id, phase)
if nargin < 3
    phase = 'all';
end

paths = PrepareStrategyRunFolders(plan_dir, task_id);
project_root = fileparts(plan_dir);

copyIfExists(fullfile(project_root, 'ig_plan1.INIVAL'), paths.initial_guess);
copyIfExists(fullfile(project_root, 'PV_plan1'), paths.initial_guess);
copyIfExists(fullfile(project_root, 'PPP_plan1'), paths.initial_guess);
copyIfExists(fullfile(project_root, 'Area_plan1'), paths.initial_guess);
copyIfExists(fullfile(project_root, 'written_initial_guess_data_plan1.mat'), paths.initial_guess);

if any(strcmpi(phase, {'all', 'optimized'}))
    result_dir = fullfile(project_root, 'results');
    if isfolder(result_dir)
        files = dir(fullfile(result_dir, '*.txt'));
        for i = 1:numel(files)
            copyfile(fullfile(files(i).folder, files(i).name), paths.optimized_variables);
        end
        writeCombined(paths.optimized_variables, task_id);
    end
end
end

function copyIfExists(src, dst)
if exist(src, 'file') == 2
    copyfile(src, dst);
end
end

function writeCombined(result_dir, task_id)
names = {'x','y','theta','v','a','phy','w','dt'};
out_file = fullfile(result_dir, sprintf('optimized_variables_task_%02d.txt', task_id));
fid = fopen(out_file, 'w');
if fid < 0
    error('Cannot create file: %s', out_file);
end
cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>

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
