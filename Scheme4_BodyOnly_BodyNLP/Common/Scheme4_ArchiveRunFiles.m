function Scheme4_ArchiveRunFiles(scheme_dir, task_id, phase)

if nargin < 3
    phase = 'all';
end

run_paths = PrepareStrategyRunFolders(scheme_dir, task_id);

% =========================================================
% Initial guess
% =========================================================
if any(strcmpi(phase, {'initial', 'all'}))

    copyIfExists( ...
        fullfile(scheme_dir, 'ig_scheme4.INIVAL'), ...
        run_paths.initial_guess);

    copyIfExists( ...
        fullfile(scheme_dir, 'PV_scheme4'), ...
        run_paths.initial_guess);

    copyIfExists( ...
        fullfile(scheme_dir, 'PPP_scheme4'), ...
        run_paths.initial_guess);

    copyIfExists( ...
        fullfile(scheme_dir, 'Area_scheme4'), ...
        run_paths.initial_guess);

    copyIfExists( ...
        fullfile(scheme_dir, 'written_initial_guess_data_scheme4.mat'), ...
        run_paths.initial_guess);
end

% =========================================================
% Optimized variables
% =========================================================
if any(strcmpi(phase, {'optimized', 'all'}))

    result_dir = fullfile(scheme_dir, 'results');

    if isfolder(result_dir)

        files = dir(fullfile(result_dir, '*.txt'));

        for i = 1:numel(files)
            copyfile( ...
                fullfile(files(i).folder, files(i).name), ...
                run_paths.optimized_variables);
        end

        writeCombined(run_paths.optimized_variables, task_id);
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

out_file = fullfile( ...
    result_dir, ...
    sprintf('optimized_variables_task_%02d.txt', task_id));

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