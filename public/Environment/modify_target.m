function modify_target(file_path)
% 修改指定任务文件中的起点/终点参数

    % ===== 1. 检查文件 =====
    if exist(file_path, 'file') ~= 2
        error('Task file does not exist: %s', file_path);
    end

    % ===== 2. 读取原文件 =====
    data = load(file_path);

    % ===== 3. 修改变量 =====

    % 起点
    %data.x0 = 3.5;
    %data.y0 = 20;
    %data.theta0 = -1.57;

    % 终点
    data.xf = 11;
    data.yf = 6;
    data.thetaf = -1.5;

    % ===== 4. 覆盖保存 =====
    save(file_path, '-struct', 'data');

    fprintf('Task updated: %s\n', file_path);

end