function UpdateUnifiedResults(output_csv, scheme_dir, result)
%UPDATEUNIFIEDRESULTS Update one result in the root 19-column CSV file.
% Each scheme uses four columns and adjacent schemes are separated by one
% empty column. Collision percentages are written with two decimal places.

scheme_folders = { ...
    'Scheme1_Full_GeoTime_LSENLP', ...
    'Scheme2_NoGeo_Time_LSENLP', ...
    'Scheme3_GeoTime_MaxNLP', ...
    'Scheme4_EqualTime_BodyOnlyNLP'};
[~, scheme_folder] = fileparts(scheme_dir);
scheme_index = find(strcmp(scheme_folder, scheme_folders), 1);
if isempty(scheme_index)
    error('UpdateUnifiedResults:UnknownScheme', ...
        'Cannot map scheme directory to results.csv: %s', scheme_dir);
end

headers = unifiedHeaders();
if exist(output_csv, 'file') == 2
    raw = readcell(output_csv, 'Delimiter', ',');
    if isempty(raw) || size(raw,2) ~= 19 || ...
            ~strcmp(string(raw{1,1}),string(headers{1}))
        error('UpdateUnifiedResults:InvalidTable', ...
            'Existing results.csv does not have the expected 19-column layout.');
    end
    rows = raw(2:end,:);
else
    rows = cell(0,19);
end

task_ids = nan(size(rows,1),1);
for r = 1:size(rows,1)
    task_ids(r) = numericCell(rows{r,1});
end
row_index = find(task_ids == result.task_id,1);
if isempty(row_index)
    new_row = cell(1,19);
    for c = [1,6,11,16]
        new_row{c} = result.task_id;
    end
    for c = [2:4,7:9,12:14,17:19]
        new_row{c} = NaN;
    end
    rows(end+1,:) = new_row;
    row_index = size(rows,1);
end

base_column = 1+(scheme_index-1)*5;
rows{row_index,base_column} = result.task_id;
rows{row_index,base_column+1} = double(result.success);
rows{row_index,base_column+2} = result.ipopt_cpu_sec;
rows{row_index,base_column+3} = round(result.collision_percent,2);

ids = nan(size(rows,1),1);
for r = 1:size(rows,1)
    ids(r) = numericCell(rows{r,1});
end
[~,order] = sort(ids);
writeUnifiedCsv(output_csv,headers,rows(order,:));
fprintf('Unified evaluation table updated: %s\n',output_csv);
end

function headers = unifiedHeaders()
headers = cell(1,19);
for scheme_index = 1:4
    base = 1+(scheme_index-1)*5;
    prefix = sprintf('scheme%d_',scheme_index);
    headers{base} = [prefix,'task_id'];
    headers{base+1} = [prefix,'success'];
    headers{base+2} = [prefix,'ipopt_cpu_time'];
    headers{base+3} = [prefix,'collision_percent'];
    if scheme_index < 4
        headers{base+4} = '';
    end
end
end

function value = numericCell(item)
if isnumeric(item) || islogical(item)
    value = double(item);
else
    value = str2double(string(item));
end
end

function writeUnifiedCsv(filename,headers,rows)
fid = fopen(filename,'w');
if fid < 0
    error('UpdateUnifiedResults:CannotWriteCsv', ...
        'Cannot create unified results file: %s',filename);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',strjoin(headers,','));
task_columns = [1,6,11,16];
success_columns = [2,7,12,17];
cpu_columns = [3,8,13,18];
collision_columns = [4,9,14,19];
for r = 1:size(rows,1)
    fields = cell(1,19);
    for c = 1:19
        if any(c == [5,10,15])
            fields{c} = '';
            continue;
        end
        value = numericCell(rows{r,c});
        if ~isfinite(value)
            fields{c} = 'NaN';
        elseif any(c == task_columns) || any(c == success_columns)
            fields{c} = sprintf('%d',round(value));
        elseif any(c == cpu_columns)
            fields{c} = sprintf('%.6f',value);
        elseif any(c == collision_columns)
            fields{c} = sprintf('%.2f',value);
        end
    end
    fprintf(fid,'%s\n',strjoin(fields,','));
end
end
