function target_nfe = ReadMatchedNfe(task_id, experiment_root)
% LEGACY: matched Nfe is now carried by the Scheme1 shared initial guess.

config_file = fullfile(experiment_root,'Scheme1_GeoTime_MaxNLP', 'Nfe_config.txt');

if exist(config_file, 'file') ~= 2
    error(['Nfe_config.txt not found. ', 'Run Scheme1 for this task first.']);
end

data = readmatrix(config_file);

if isempty(data) || size(data,2) < 2
    error('Invalid Nfe_config.txt format.');
end

idx = find(data(:,1) == task_id, 1);

if isempty(idx)
    error('No matched Nfe found for task_id=%d. Run Scheme1 first.', task_id);
end

target_nfe = round(data(idx,2));

if target_nfe < 2
    error('Invalid matched Nfe=%d for task_id=%d.', target_nfe, task_id);
end

fprintf('Matched Nfe loaded: task_id=%d, Nfe=%d\n',task_id, target_nfe);

end
