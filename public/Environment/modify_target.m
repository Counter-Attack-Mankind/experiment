function modify_target()
% 修改 Data_test/*.mat 中的起点/终点参数
% 只需要改 task_id 一次即可

    % ===== 1. 任务编号 =====
    task_id = 1;
    file_path = fullfile('Data_test', [num2str(task_id), '.mat']);

    % ===== 2. 读取原文件 =====
    data = load(file_path);

    % ===== 3. 修改变量 =====

     % data.x0 = 22;
    %  data.y0 = 2;
    % data.theta0 = 1.57;

       data.xf = 26;
      % data.yf = 13;
       data.thetaf = 3.14;

    % ===== 4. 保存（覆盖原文件） =====
    save(file_path, '-struct', 'data');

    % ===== 5. 可视化检查 =====
    DrawEnvironment(file_path);

end