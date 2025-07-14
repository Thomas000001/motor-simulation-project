%% 主程式：齒輪直流馬達能耗分析（改進版）
% 文件：power_energy_analysis_improved.m
% 描述：分析齒輪直流馬達在動態應用中的能耗
% 作者：[您的名字]
% 日期：2024

function power_energy_analysis_improved()
    % clear; clc; close all;
    
    % 設定警告
    warning('on', 'all');
    
    %% 1. 初始化與參數設定
    fprintf('=== 齒輪直流馬達能耗分析系統 v2.0 ===\n');
    fprintf('開始時間：%s\n\n', datestr(now));
    
    % 選擇馬達類型
    motorType = '60rpm'; % 可選：'60rpm' 或 '300rpm'
    
    % 獲取系統參數
    params = getMotorParameters(motorType);
    
    % 驗證參數
    validateParameters(params);
    
    %% 2. 仿真設定
    % 定義要測試的電壓範圍
    voltages_to_test = 5.0:-0.2:0.6;
    num_voltages = length(voltages_to_test);
    
    % 初始化結果結構
    results = initializeResults(num_voltages);
    
    %% 3. 檢查並設定平行運算環境
    useParallel = setupParallelEnvironment(num_voltages);
    
    %% 4. 執行多電壓點仿真
    fprintf('開始進行多電壓點的穩態仿真...\n');
    fprintf('測試電壓範圍：%.1fV 到 %.1fV\n', max(voltages_to_test), min(voltages_to_test));
    fprintf('總共 %d 個測試點\n\n', num_voltages);
    
    % 主要仿真迴圈
    if useParallel
        % 平行運算 - 使用臨時變數來儲存結果
        temp_results = cell(num_voltages, 1);
        parfor i = 1:num_voltages
            temp_results{i} = simulateSingleVoltage(voltages_to_test(i), params, i);
        end
        % 將 cell array 轉換回結構體陣列
        for i = 1:num_voltages
            results(i) = temp_results{i};
        end
    else
        % 序列運算
        progressbar = waitbar(0, '正在進行仿真...', 'Name', '仿真進度');
        for i = 1:num_voltages
            results(i) = simulateSingleVoltage(voltages_to_test(i), params, i);
            waitbar(i/num_voltages, progressbar, sprintf('完成 %d/%d', i, num_voltages));
        end
        close(progressbar);
    end
    
    %% 5. 後處理與分析
    fprintf('\n開始後處理與分析...\n');
    
    % 整理結果
    [avg_current, avg_power, avg_speed, energy_metric, efficiency] = processResults(results, voltages_to_test);
    
    % 找出最佳操作點
    [optimal_voltage, optimal_efficiency] = findOptimalOperatingPoint(voltages_to_test, efficiency, energy_metric);
    
    fprintf('\n最佳操作點：\n');
    fprintf('  電壓：%.2f V\n', optimal_voltage);
    fprintf('  效率：%.2f %%\n', optimal_efficiency * 100);
    
    %% 6. 視覺化結果
    plotResults(voltages_to_test, avg_current, avg_power, avg_speed, energy_metric, efficiency, params);
    
    %% 7. 儲存結果
    saveResults(results, params, voltages_to_test, motorType);
    
    fprintf('\n分析完成！\n');
    fprintf('結束時間：%s\n', datestr(now));
end

%% 參數設定函數
function params = getMotorParameters(motorType)
    % 基礎電氣參數
    params.motorType = motorType;
    params.R_m = 38;                % 馬達電阻 (Ohm)
    params.L_m = 0.015e-3;          % 馬達電感 (H)
    params.kt = 0.194 * 600;        % 力矩常數 (N·m/A)
    params.kb = 0.0532 / 250;       % 反電動勢常數 (V/(rad/s))
    params.J_m = 1e-7;              % 馬達慣量 (kg·m²)
    params.v_m = 1.25012e-3;        % 馬達黏性阻尼 (N·m·s/rad)
    params.Ts_motor = 0.00485;      % 馬達啟動最大靜摩擦力矩 (N·m)
    
    % 根據馬達類型調整參數
    switch motorType
        case '60rpm'
            params.n_gear = 699.55;     % 齒輪比
            params.J_gear = 5e-4/3;     % 齒輪慣量
        case '300rpm'
            params.n_gear = 136.02;     % 齒輪比
            params.R_m = 38 * 1.1;      % 調整電阻
            params.J_gear = 5e-4/3;     % 齒輪慣量
        otherwise
            error('未知的馬達類型：%s', motorType);
    end
    
    % 機械系統參數
    params.r = 6;                   % 曲柄半徑 (mm)
    params.l = 22;                  % 連桿長度 (mm)
    params.k_spring = 0.1031979802 * 1.5; % 彈簧常數 (N/mm)
    params.v_track = 0.1;           % 軌道黏性摩擦係數
    params.dist_to_spring = 5.566;  % 到彈簧的距離 (mm)
    params.max_compression = 4.5;   % 彈簧最大壓縮量 (mm)
    
    % 熱力學參數（預留）
    params.alpha = 0.0039;          % 銅的電阻溫度係數
    params.T_ref = 25;              % 參考溫度 (°C)
end

%% 參數驗證函數
function validateParameters(params)
    % 檢查物理限制
    if params.max_compression > (params.l - params.r)
        error('最大壓縮量超過機構物理限制！');
    end
    
    if params.J_gear < params.J_m
        warning('齒輪慣量通常應大於馬達慣量');
    end
    
    if params.r >= params.l
        error('曲柄半徑必須小於連桿長度！');
    end
    
    % 檢查數值範圍
    if params.kt <= 0 || params.kb <= 0
        error('馬達常數必須為正值！');
    end
    
    fprintf('參數驗證通過\n');
end

%% 初始化結果結構
function results = initializeResults(num_voltages)
    % 創建一個包含所有欄位的範本結構
    template = struct('voltage', 0, ...
                     'avg_current', 0, ...
                     'avg_power', 0, ...
                     'avg_speed', 0, ...
                     'energy_metric', 0, ...
                     'efficiency', 0, ...
                     'convergence_time', 0, ...
                     'cycle_duration', 0, ...
                     'success', false, ...
                     'error_msg', '');
    
    % 使用 repmat 創建結構體陣列
    results = repmat(template, num_voltages, 1);
end

%% 設定平行運算環境
function useParallel = setupParallelEnvironment(num_tasks)
    useParallel = false;
    
    if ~isempty(ver('parallel')) && num_tasks > 4
        try
            poolobj = gcp('nocreate');
            if isempty(poolobj)
                numWorkers = min(num_tasks, maxNumCompThreads);
                parpool('local', numWorkers);
                fprintf('已啟動平行運算池，使用 %d 個工作執行緒\n', numWorkers);
            else
                fprintf('使用現有平行運算池，%d 個工作執行緒\n', poolobj.NumWorkers);
            end
            useParallel = true;
        catch ME
            warning('無法啟動平行運算：%s', ME.message);
            fprintf('將使用序列運算\n');
        end
    else
        fprintf('使用序列運算模式\n');
    end
end

%% 單一電壓仿真函數
function result = simulateSingleVoltage(V_input, params, idx)
    % 初始化完整的結果結構
    result = struct('voltage', V_input, ...
                   'avg_current', 0, ...
                   'avg_power', 0, ...
                   'avg_speed', 0, ...
                   'energy_metric', 0, ...
                   'efficiency', 0, ...
                   'convergence_time', 0, ...
                   'cycle_duration', 0, ...
                   'success', false, ...
                   'error_msg', '');
    
    try
        % 動態決定仿真時間
        t_end = determineSimulationTime(V_input, params);
        tspan = [0, t_end];
        
        % 初始條件 [theta_m; dtheta_m; I_m]
        x0 = [0; 0; 0];
        
        % ODE 求解器選項
        options = odeset('RelTol', 1e-6, ...
                        'AbsTol', [1e-8, 1e-8, 1e-9], ...
                        'MaxStep', 0.01, ...
                        'Stats', 'off');
        
        % 求解 ODE
        [t, x] = ode15s(@(t,x) motorDynamics(t, x, V_input, params), tspan, x0, options);
        
        % 分析穩態結果
        [success, cycleData] = analyzeSteadyState(t, x, V_input, params);
        
        if success
            % 計算性能指標
            metrics = calculatePerformanceMetrics(cycleData, V_input, params);
            
            % 更新結果結構
            result.voltage = V_input;
            result.avg_current = metrics.avg_current;
            result.avg_power = metrics.avg_power;
            result.avg_speed = metrics.avg_speed;
            result.energy_metric = metrics.energy_metric;
            result.efficiency = metrics.efficiency;
            result.success = true;
            result.convergence_time = cycleData.convergence_time;
            result.cycle_duration = cycleData.cycle_duration;
            result.error_msg = '';
            
            fprintf('電壓 %.1fV 仿真完成 - 平均電流：%.2f mA, 效率：%.1f%%\n', ...
                    V_input, result.avg_current, result.efficiency * 100);
        else
            result.error_msg = '未達到穩態或週期不完整';
            fprintf('電壓 %.1fV 仿真失敗：%s\n', V_input, result.error_msg);
        end
        
    catch ME
        result.error_msg = ME.message;
        fprintf('電壓 %.1fV 仿真錯誤：%s\n', V_input, ME.message);
    end
end

%% 動態決定仿真時間
function t_end = determineSimulationTime(V_input, params)
    % 根據電壓和系統參數估計達到穩態所需時間
    if V_input < 1.5
        t_end = 120.0;  % 極低電壓需要很長時間
    elseif V_input < 2.5
        t_end = 120.0;  % 低電壓
    elseif V_input < 3.5
        t_end = 120.0;  % 中等電壓
    else
        t_end = 120.0;   % 高電壓快速達到穩態
    end
end

%% 馬達動力學方程
function dxdt = motorDynamics(t, x, V_input, params)
    % 解包狀態變量
    theta_m = x(1);     % 馬達角度
    dtheta_m = x(2);    % 馬達角速度
    I_m = x(3);         % 馬達電流
    
    % 解包參數
    kt = params.kt; kb = params.kb; R_m = params.R_m; L_m = params.L_m;
    J_motor = params.J_m; J_gear = params.J_gear; n_gear = params.n_gear;
    v_motor = params.v_m; Ts_motor = params.Ts_motor;
    r = params.r; l = params.l; k_spring = params.k_spring; v_track = params.v_track;
    dist_to_spring = params.dist_to_spring; max_compression = params.max_compression;
    
    % 計算曲柄角度
    theta_crank = theta_m / n_gear;
    
    % 計算滑塊位置和速度
    [x_slider, v_slider, jacobian] = calculateSliderKinematics(theta_crank, dtheta_m/n_gear, r, l);
    
    % 計算彈簧力
    F_spring = calculateSpringForce(x_slider, l, r, dist_to_spring, max_compression, k_spring);
    
    % 計算軌道摩擦力
    F_track_friction = -v_track * v_slider;
    
    % 總負載力
    F_total = F_spring + F_track_friction;
    
    % 將負載力轉換為馬達軸上的力矩
    % T_load = (F_total * jacobian) / n_gear;
    T_load = (F_total * jacobian);
    
    % 馬達產生的力矩
    T_motor = kt * I_m;
    
    % 摩擦力矩計算
    T_friction = calculateFriction(dtheta_m, T_motor, Ts_motor, v_motor);
    
    % 總慣量
    J_total = J_motor + J_gear;
    
    % 淨力矩
    T_net = T_motor - T_friction + T_load;
    
    % 狀態方程
    dx1dt = dtheta_m;                                           % dθ/dt
    dx2dt = T_net / J_total;                                    % dω/dt
    dx3dt = (V_input - R_m * I_m - kb * dtheta_m) / L_m;      % dI/dt
    
    dxdt = [dx1dt; dx2dt; dx3dt];
end

%% 計算滑塊運動學
function [x_slider, v_slider, jacobian] = calculateSliderKinematics(theta_crank, omega_crank, r, l)
    sin_theta = sin(theta_crank);
    cos_theta = cos(theta_crank);
    
    % 滑塊位置
    term_in_sqrt = l^2 - (r * sin_theta)^2;
    term_in_sqrt = max(term_in_sqrt, 0); % 避免數值錯誤
    x_slider = r * cos_theta + sqrt(term_in_sqrt);
    
    % 雅可比計算（避免奇異點）
    if term_in_sqrt < 1e-10
        jacobian = -r * sin_theta;
    else
        sqrt_term = sqrt(term_in_sqrt);
        jacobian = -r * sin_theta - (r^2 * sin_theta * cos_theta) / sqrt_term;
    end
    
    % 滑塊速度
    v_slider = omega_crank * jacobian;
end

%% 計算彈簧力
function F_spring = calculateSpringForce(x_slider, l, r, dist_to_spring, max_compression, k_spring)
    x_min = l - r;
    distance = x_slider - x_min;
    compression = distance - dist_to_spring;
    
    if compression > 0
        if compression > max_compression
            F_spring = -k_spring * max_compression; % 限制最大壓縮
        else
            F_spring = -k_spring * compression;
        end
    else
        F_spring = 0; % 無壓縮
    end
end

%% 計算摩擦力矩
function T_friction = calculateFriction(omega, T_motor, Ts_static, v_viscous)
    epsilon = 1e-6; % 速度閾值
    
    if abs(omega) < epsilon
        % 靜止狀態
        if abs(T_motor) < Ts_static
            T_friction = T_motor; % 靜摩擦
        else
            T_friction = sign(T_motor) * Ts_static * 0.9; % 克服靜摩擦
        end
    else
        % 運動狀態：庫侖摩擦 + 黏性摩擦
        T_friction = sign(omega) * Ts_static * 0.8 + v_viscous * omega;
    end
end

%% 分析穩態結果
function [success, cycleData] = analyzeSteadyState(t, x, V_input, params)
    success = false;
    cycleData = struct();
    
    % 忽略瞬態響應
    transient_time = 2.0;
    stable_idx = t > transient_time;
    
    if sum(stable_idx) < 100
        return; % 數據不足
    end
    
    t_stable = t(stable_idx);
    theta_m_stable = x(stable_idx, 1);
    dtheta_m_stable = x(stable_idx, 2);
    I_m_stable = x(stable_idx, 3);
    
    % 找到完整週期
    try
        [t_cycle, cycle_indices] = findCompleteCycles(t_stable, theta_m_stable, params.n_gear);
        
        if isempty(t_cycle)
            return; % 未找到完整週期
        end
        
        % 提取最後一個完整週期的數據
        cycleData.t = t_cycle;
        cycleData.theta_m = theta_m_stable(cycle_indices);
        cycleData.dtheta_m = dtheta_m_stable(cycle_indices);
        cycleData.I_m = I_m_stable(cycle_indices);
        cycleData.convergence_time = t_cycle(1);
        cycleData.cycle_duration = t_cycle(end) - t_cycle(1);
        
        success = true;
    catch
        return; % 週期檢測失敗
    end
end

%% 找到完整週期
function [t_cycle, cycle_indices] = findCompleteCycles(t, theta_m, n_gear)
    % 將馬達角度轉換為曲柄角度並歸一化
    theta_crank = mod(theta_m / n_gear, 2*pi);
    
    % 找到零交叉點（從2π到0的跳變）
    zero_crossings = find(diff(theta_crank) < -pi);
    
    if length(zero_crossings) < 2
        t_cycle = [];
        cycle_indices = [];
        return;
    end
    
    % 選擇最後3個完整週期
    start_idx = zero_crossings(end-3) + 1;
    end_idx = zero_crossings(end);
    
    cycle_indices = start_idx:end_idx;
    t_cycle = t(cycle_indices);
end

%% 計算性能指標(平均電流，平均功耗，平均能耗，平均轉速等)
function metrics = calculatePerformanceMetrics(cycleData, V_input, params)
    % 基本平均值
    avg_I = mean(cycleData.I_m);
    avg_omega = mean(cycleData.dtheta_m);
    
    % 計算功率
    input_power = V_input * avg_I;
    
    % 計算機械功率（近似）
    T_motor = params.kt * cycleData.I_m;
    mechanical_power = mean(T_motor .* cycleData.dtheta_m);
    
    % 效率
    if input_power > 0
        efficiency = mechanical_power / input_power;
        efficiency = max(0, min(1, efficiency)); % 限制在0-1之間
    else
        efficiency = 0;
    end
    
    % 轉速（rev/s）
    avg_speed_rev_s = avg_omega / (2*pi * params.n_gear);
    
    % 能耗指標
    if avg_speed_rev_s > 1e-6
        energy_metric = (V_input * avg_I * 1000) / avg_speed_rev_s; % V·mA/(rev/s)
    else
        energy_metric = inf;
    end
    
    % 封裝結果
    metrics.avg_current = avg_I * 1000; % 轉換為 mA
    metrics.avg_power = input_power * 1000; % 轉換為 V·mA
    metrics.avg_speed = avg_speed_rev_s;
    metrics.energy_metric = energy_metric;
    metrics.efficiency = efficiency;
end

%% 處理所有結果
function [avg_current, avg_power, avg_speed, energy_metric, efficiency] = processResults(results, voltages)
    n = length(voltages);
    avg_current = zeros(n, 1);
    avg_power = zeros(n, 1);
    avg_speed = zeros(n, 1);
    energy_metric = zeros(n, 1);
    efficiency = zeros(n, 1);
    
    for i = 1:n
        if results(i).success
            avg_current(i) = results(i).avg_current;
            avg_power(i) = results(i).avg_power;
            avg_speed(i) = results(i).avg_speed;
            energy_metric(i) = results(i).energy_metric;
            efficiency(i) = results(i).efficiency;
        else
            % 標記失敗的點
            avg_current(i) = NaN;
            avg_power(i) = NaN;
            avg_speed(i) = NaN;
            energy_metric(i) = NaN;
            efficiency(i) = NaN;
        end
    end
end

%% 找出最佳操作點
function [optimal_voltage, optimal_efficiency] = findOptimalOperatingPoint(voltages, efficiency, energy_metric)
    % 移除無效數據
    valid_idx = ~isnan(efficiency) & ~isinf(energy_metric);
    
    if sum(valid_idx) == 0
        optimal_voltage = NaN;
        optimal_efficiency = NaN;
        return;
    end
    
    % 找出效率最高的點
    [optimal_efficiency, idx] = max(efficiency(valid_idx));
    valid_voltages = voltages(valid_idx);
    optimal_voltage = valid_voltages(idx);
end

%% 繪製結果圖表
function plotResults(voltages, current, power, speed, energy_metric, efficiency, params)
    % 創建主要分析圖
    figure('Name', '馬達性能分析', 'Position', [50, 50, 1400, 900]);
    
    % 子圖1：電流
    subplot(1, 4, 1);
    plot(voltages, current, 'b-o', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', 'b');
    xlabel('輸入電壓 (V)');
    ylabel('平均電流 (mA)');
    title('平均電流 vs. 輸入電壓');
    grid on;
    addDataLabels(voltages, current, '%.1f');
    
    % 子圖2：功率
    subplot(1, 4, 2);
    plot(voltages, power, 'r-o', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', 'r');
    xlabel('輸入電壓 (V)');
    ylabel('平均功率 (V·mA)');
    title('平均功率 vs. 輸入電壓');
    grid on;
    
    % 子圖3：轉速
    subplot(1, 4, 3);
    plot(voltages, speed, 'g-o', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', 'g');
    xlabel('輸入電壓 (V)');
    ylabel('平均轉速 (rev/s)');
    title('平均轉速 vs. 輸入電壓');
    grid on;
    
    % 子圖4：能耗指標
    subplot(1, 4, 4);
    valid_em = energy_metric(~isinf(energy_metric));
    valid_v = voltages(~isinf(energy_metric));
    plot(valid_v, valid_em, 'm-o', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', 'm');
    xlabel('輸入電壓 (V)');
    ylabel('能耗指標 (V·mA/(rev/s))');
    title('能耗指標 vs. 輸入電壓');
    grid on;
    
    % % 子圖5：效率
    % subplot(2, 3, 5);
    % plot(voltages, efficiency*100, 'k-o', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', 'k');
    % xlabel('輸入電壓 (V)');
    % ylabel('效率 (%)');
    % title('系統效率 vs. 輸入電壓');
    % grid on;
    % ylim([0, max(efficiency*100)*1.1]);
    % 
    % % 子圖6：綜合性能圖
    % subplot(2, 3, 6);
    % yyaxis left;
    % plot(voltages, power, 'r-', 'LineWidth', 2);
    % ylabel('功率 (V·mA)');
    % yyaxis right;
    % plot(voltages, efficiency*100, 'b-', 'LineWidth', 2);
    % ylabel('效率 (%)');
    % xlabel('輸入電壓 (V)');
    % title('功率與效率');
    % grid on;
    % legend('功率', '效率', 'Location', 'best');
    
    % 總標題
    sgtitle(sprintf('馬達性能分析 - %s 配置', params.motorType), 'FontSize', 16);
    
    % % 創建詳細分析圖
    % figure('Name', '詳細性能分析', 'Position', [100, 150, 1200, 700]);
    % 
    % % 3D 性能圖
    % subplot(1, 2, 1);
    % [V_mesh, S_mesh] = meshgrid(voltages, linspace(min(speed), max(speed), 50));
    % EM_interp = griddata(voltages, speed, energy_metric, V_mesh, S_mesh, 'cubic');
    % surf(V_mesh, S_mesh, EM_interp);
    % xlabel('電壓 (V)');
    % ylabel('轉速 (rev/s)');
    % zlabel('能耗指標');
    % title('能耗指標 3D 圖');
    % colorbar;
    % view(45, 30);
    % 
    % % 效率等高線圖
    % subplot(1, 2, 2);
    % [V_mesh2, C_mesh] = meshgrid(voltages, linspace(min(current), max(current), 50));
    % E_interp = griddata(voltages, current, efficiency*100, V_mesh2, C_mesh, 'cubic');
    % contourf(V_mesh2, C_mesh, E_interp, 20);
    % xlabel('電壓 (V)');
    % ylabel('電流 (mA)');
    % title('效率等高線圖 (%)');
    % colorbar;
end

%% 添加數據標籤
function addDataLabels(x, y, format)
    % 在選定的數據點上添加標籤
    n = length(x);
    step = max(1, floor(n/10)); % 最多顯示10個標籤
    
    for i = 1:step:n
        if ~isnan(y(i))
            text(x(i), y(i), sprintf(format, y(i)), ...
                 'VerticalAlignment', 'bottom', ...
                 'HorizontalAlignment', 'center', ...
                 'FontSize', 8);
        end
    end
end

%% 儲存結果
function saveResults(results, params, voltages, motorType)
    % 創建結果目錄
    if ~exist('results', 'dir')
        mkdir('results');
    end
    
    % 生成檔案名稱
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    mat_filename = fullfile('results', sprintf('motor_analysis_%s_%s.mat', motorType, timestamp));
    excel_filename = fullfile('results', sprintf('motor_analysis_%s_%s.xlsx', motorType, timestamp));
    
    % 儲存 MAT 檔案
    save(mat_filename, 'results', 'params', 'voltages');
    fprintf('\n結果已儲存至：%s\n', mat_filename);
    
    % 準備 Excel 數據
    excel_data = table();
    excel_data.Voltage = voltages(:);
    
    for i = 1:length(results)
        excel_data.Current_mA(i) = results(i).avg_current;
        excel_data.Power_VmA(i) = results(i).avg_power;
        excel_data.Speed_revps(i) = results(i).avg_speed;
        excel_data.EnergyMetric(i) = results(i).energy_metric;
        excel_data.Efficiency_percent(i) = results(i).efficiency * 100;
        excel_data.ConvergenceTime_s(i) = results(i).convergence_time;
        excel_data.CycleDuration_s(i) = results(i).cycle_duration;
        excel_data.Success(i) = results(i).success;
    end
    
    % 寫入 Excel 檔案
    try
        writetable(excel_data, excel_filename);
        fprintf('Excel 報告已儲存至：%s\n', excel_filename);
        
        % 創建摘要表
        summary_filename = fullfile('results', sprintf('summary_%s_%s.txt', motorType, timestamp));
        writeSummaryReport(summary_filename, results, params, voltages);
        
    catch ME
        warning('無法寫入 Excel 檔案：%s', ME.message);
    end
end

%% 撰寫摘要報告
function writeSummaryReport(filename, results, params, voltages)
    fid = fopen(filename, 'w');
    if fid == -1
        warning('無法創建摘要報告檔案');
        return;
    end
    
    fprintf(fid, '====================================\n');
    fprintf(fid, '馬達能耗分析摘要報告\n');
    fprintf(fid, '====================================\n');
    fprintf(fid, '報告生成時間：%s\n\n', datestr(now));
    
    fprintf(fid, '【系統參數】\n');
    fprintf(fid, '馬達類型：%s\n', params.motorType);
    fprintf(fid, '電阻：%.2f Ohm\n', params.R_m);
    fprintf(fid, '電感：%.3e H\n', params.L_m);
    fprintf(fid, '力矩常數：%.3f N·m/A\n', params.kt);
    fprintf(fid, '反電動勢常數：%.6f V/(rad/s)\n', params.kb);
    fprintf(fid, '齒輪比：%.2f\n', params.n_gear);
    fprintf(fid, '彈簧常數：%.6f N/mm\n\n', params.k_spring);
    
    fprintf(fid, '【測試範圍】\n');
    fprintf(fid, '電壓範圍：%.1f V 到 %.1f V\n', min(voltages), max(voltages));
    fprintf(fid, '測試點數：%d\n\n', length(voltages));
    
    % 找出最佳操作點
    valid_results = [results.success];
    if any(valid_results)
        efficiencies = [results.efficiency];
        [best_eff, best_idx] = max(efficiencies);
        
        fprintf(fid, '【最佳操作點】\n');
        fprintf(fid, '電壓：%.2f V\n', results(best_idx).voltage);
        fprintf(fid, '效率：%.2f %%\n', best_eff * 100);
        fprintf(fid, '電流：%.2f mA\n', results(best_idx).avg_current);
        fprintf(fid, '功率：%.2f V·mA\n', results(best_idx).avg_power);
        fprintf(fid, '轉速：%.3f rev/s\n', results(best_idx).avg_speed);
        fprintf(fid, '能耗指標：%.2f V·mA/(rev/s)\n\n', results(best_idx).energy_metric);
        
        % 統計資訊
        fprintf(fid, '【統計資訊】\n');
        fprintf(fid, '成功仿真數：%d / %d\n', sum(valid_results), length(results));
        fprintf(fid, '平均效率：%.2f %%\n', mean(efficiencies(valid_results)) * 100);
        fprintf(fid, '效率範圍：%.2f %% - %.2f %%\n', ...
                min(efficiencies(valid_results)) * 100, ...
                max(efficiencies(valid_results)) * 100);
    end
    
    fclose(fid);
    fprintf('摘要報告已儲存至：%s\n', filename);
end

%% 輔助函數：進度條顯示（用於非平行運算）
function updateProgress(current, total, start_time)
    % 計算進度和預估剩餘時間
    progress = current / total;
    elapsed_time = toc(start_time);
    
    if current > 0
        estimated_total = elapsed_time / progress;
        remaining_time = estimated_total - elapsed_time;
        
        fprintf('\r進度：%d/%d (%.1f%%) - 已用時間：%.1fs - 預估剩餘：%.1fs', ...
                current, total, progress*100, elapsed_time, remaining_time);
    end
    
    if current == total
        fprintf('\n');
    end
end