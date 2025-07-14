function ode45_load_02310_simulation_discrete_2()
    %% 改进的电机-连杆-弹簧系统仿真
    % 增强的代码结构，更好的参数管理和错误处理
    
    % clear; clc; close all;
    
    %% 1. 参数初始化 - 使用结构体统一管理
    params = initialize_parameters();
    
    %% 2. 仿真配置
    sim_config = setup_simulation_config();
    
    %% 3. 执行仿真
    [t, x] = run_simulation(params, sim_config);
    
    %% 4. 结果分析和可视化
    analyze_and_plot_results(t, x, params, sim_config);
    
    %% 5. 性能评估
    performance_metrics = calculate_performance_metrics(t, x, params, sim_config);
    display_performance_summary(performance_metrics);
end

function params = initialize_parameters()
    %% 系统参数初始化
    
    % 连杆机构参数
    params.linkage.r = 6;                    % 曲柄半径 (mm)
    params.linkage.l = 22;                   % 连杆长度 (mm)
    params.linkage.dist_to_spring = 5.566;   % 到弹簧的距离 (mm)
    
    % 弹簧参数
    params.spring.k = 0.1031979802 * 1.5;    % 彈簧常數 (N/mm)
    params.spring.max_compression = 4.5;     % 最大压缩量 (mm)
    
    % 电机电气参数
    params.motor.R_m = 38;                   % 60rpm电阻 (Ω)
    params.motor.L_m = 0.015e-3;             % 电感 (H)
    params.motor.kt = 0.194 * 600;           % 转矩常数 (Nm/A)
    params.motor.kb = 0.0532 / 250;          % 反电动势常数 (V·s/rad)
    
    % 机械参数
    params.motor.J_m = 1e-7;                 % 电机转动惯量 (kg·m²)
    params.motor.J_gear = 5e-4 / 3;          % 齿轮箱转动惯量 (kg·m²)
    params.motor.n_gear = 699.55;            % 齿轮比
    
    % 摩擦参数
    params.friction.v_motor = 1.25012e-3;    % 电机粘性阻尼
    params.friction.v_track = 0.2;           % 轨道粘性摩擦系数
    params.friction.Fc_track = 0.1;          % 轨道库伦摩擦力 (N)
    params.friction.Ts_motor = 0.00485;      % 电机最大静摩擦力矩 (Nm)
    
    % 热参数
    params.thermal.alpha = 0.0039;           % 铜的电阻温度系数
    params.thermal.T_ref = 25;               % 参考温度 (°C)
    
    % 数值计算参数
    params.numerical.epsilon = 1e-6;         % 速度阈值
    params.numerical.static_friction_factor = 0.9;  % 静摩擦系数
    params.numerical.kinetic_friction_factor = 0.8; % 动摩擦系数
end

function config = setup_simulation_config()
    %% 仿真配置设置
    
    config.voltages = 5.0:-0.2:0.6;          % 电压阶跃序列
    config.time_per_step = 12.5;             % 每步时间 (s)
    config.initial_conditions = [0; 0; 0];   % 初始条件 [theta_m; omega_m; I_m]
    
    % ODE求解器選項
    % config.ode_options = odeset('RelTol', 1e-3, 'AbsTol', [1e-5 1e-5 1e-4], ...
    %                             'MaxStep', 0.01, 'Stats', 'on');
    config.ode_options = odeset('RelTol', 1e-3, 'AbsTol', [1e-5 1e-5 1e-4], ...
                                'Stats', 'on');
    %   MaxStep ：最大步長限制
    %   Stats ：顯示求解器性能統計，成功步數/失敗步數/函數評估次數/雅可比評估次數
    
    % 计算总仿真时间
    config.total_duration = config.time_per_step * length(config.voltages);
    config.tspan = [0, config.total_duration];
end

function [t, x] = run_simulation(params, config)
    %% 執行仿真
    
    fprintf('開始馬達-連桿-彈簧系統仿真...\n');
    fprintf('電壓範圍: %.1fV - %.1fV\n', max(config.voltages), min(config.voltages));
    fprintf('仿真時長: %.1fs\n', config.total_duration);
    
    tic;
    [t, x] = ode45(@(t,x) system_dynamics(t, x, config, params), ...
                   config.tspan, config.initial_conditions, config.ode_options);
    simulation_time = toc;
    
    fprintf('仿真完成！用時: %.2fs\n', simulation_time);
    fprintf('數據點數: %d\n', length(t));
end

function dxdt = system_dynamics(t, x, config, params)
    %% 系统动力学方程
    
    % 状态变量
    theta_m = x(1);      % 电机角度 (rad)
    omega_m = x(2);      % 电机角速度 (rad/s)
    I_m = x(3);          % 电机电流 (A)
    
    % 获取当前电压
    V_input = get_voltage_at_time(t, config.voltages, config.time_per_step);
    
    % 计算负载力矩
    T_load = calculate_load_torque(theta_m, omega_m, params);
    
    % 计算摩擦力矩
    T_friction = calculate_friction_torque(omega_m, I_m, params);
    
    % 电机产生的力矩
    T_motor = params.motor.kt * I_m;
    
    % 系统动力学方程
    J_total = params.motor.J_m + params.motor.J_gear;
    
    % 状态导数
    dtheta_dt = omega_m;
    domega_dt = (T_motor - T_friction + T_load) / J_total;
    
    % 电路方程
    V_emf = params.motor.kb * omega_m;
    dI_dt = (V_input - params.motor.R_m * I_m - V_emf) / params.motor.L_m;
    
    dxdt = [dtheta_dt; domega_dt; dI_dt];
end

function T_load = calculate_load_torque(theta_m, omega_m, params)
    %% 计算负载力矩（弹簧力 + 轨道摩擦）
    
    % 曲柄角度
    theta_crank = theta_m / params.motor.n_gear;
    
    % 滑块位置计算
    x_slider = params.linkage.r * cos(theta_crank) + ...
               sqrt(params.linkage.l^2 - (params.linkage.r * sin(theta_crank))^2);
    x_min = params.linkage.l - params.linkage.r;
    distance = x_slider - x_min;
    
    % 弹簧力计算
    compression = distance - params.linkage.dist_to_spring;
    F_spring = 0;
    if compression > 0
        compression = min(compression, params.spring.max_compression);
        F_spring = -params.spring.k * compression;  % 负号表示阻力
    end
    
    % 雅可比计算
    term_in_sqrt = params.linkage.l^2 - (params.linkage.r * sin(theta_crank))^2;
    term_in_sqrt = max(term_in_sqrt, 0);  % 避免负值
    jacobian = -params.linkage.r * sin(theta_crank) - ...
               (params.linkage.r^2 * sin(theta_crank) * cos(theta_crank)) / sqrt(term_in_sqrt);
    
    % 滑块速度
    omega_crank = omega_m / params.motor.n_gear;
    v_slider = omega_crank * jacobian;
    
    % 轨道摩擦力
    F_track_friction = -(params.friction.Fc_track * sign(v_slider) + ...
                         params.friction.v_track * v_slider);
    
    % 总负载力矩(包含機構的摩擦力)
    T_crank = (F_spring + F_track_friction) * jacobian;
    T_load = T_crank / params.motor.n_gear;
end

function T_friction = calculate_friction_torque(omega_m, I_m, params)
    %% 计算摩擦力矩
    
    T_motor_gen = params.motor.kt * I_m;
    epsilon = params.numerical.epsilon;
    
    if abs(omega_m) < epsilon
        % 静止状态
        if abs(T_motor_gen) < params.friction.Ts_motor
            T_friction = T_motor_gen;  % 静摩擦平衡
        else
            T_friction = params.friction.Ts_motor * ...
                        params.numerical.static_friction_factor * sign(omega_m);
        end
    else
        % 运动状态
        T_friction = params.friction.Ts_motor * ...
                    params.numerical.kinetic_friction_factor * sign(omega_m) + ...
                    params.friction.v_motor * omega_m;
    end
end

function V_current = get_voltage_at_time(t, voltages, step_duration)
    %% 获取当前时间对应的电压值
    idx = min(floor(t / step_duration) + 1, length(voltages));
    V_current = voltages(idx);
end

function analyze_and_plot_results(t, x, params, config)
    %% 结果分析和可视化
    
    % 计算衍生量
    theta_m = x(:, 1);
    omega_m = x(:, 2);
    I_m = x(:, 3);
    
    % 转换为工程单位
    revolutions = theta_m / (2 * pi * params.motor.n_gear);
    rpm = omega_m * 60 / (2 * pi * params.motor.n_gear);
    current_mA = I_m * 1000;
    
    % 计算功率
    voltage_profile = arrayfun(@(time) get_voltage_at_time(time, config.voltages, config.time_per_step), t);
    power_W = voltage_profile .* I_m;
    
    % 创建图形
    figure('Name', '电机系统动态响应分析', 'Position', [100, 100, 1200, 800]);
    
    % 子图1：转数
    subplot(2, 2, 1);
    plot(t, revolutions, 'b-', 'LineWidth', 1.5);
    title('电机转数响应');
    xlabel('时间 (s)');
    ylabel('转数 (rev)');
    grid on;
    add_voltage_markers(config);
    
    % 子图2：转速
    subplot(2, 2, 2);
    plot(t, rpm, 'g-', 'LineWidth', 1.5);
    title('电机转速响应');
    xlabel('时间 (s)');
    ylabel('转速 (rpm)');
    grid on;
    add_voltage_markers(config);
    
    % 子图3：电流
    subplot(2, 2, 3);
    plot(t, current_mA, 'r-', 'LineWidth', 1.5);
    title('电机电流响应');
    xlabel('时间 (s)');
    ylabel('电流 (mA)');
    grid on;
    add_voltage_markers(config);
    
    % 子图4：功率
    subplot(2, 2, 4);
    plot(t, power_W, 'm-', 'LineWidth', 1.5);
    title('电机功率响应');
    xlabel('时间 (s)');
    ylabel('功率 (W)');
    grid on;
    add_voltage_markers(config);
    
    sgtitle('电机-连杆-弹簧负载系统阶跃响应分析');
end

function add_voltage_markers(config)
    %% 添加电压切换标记
    hold on;
    for i = 1:(length(config.voltages)-1)
        xline(i * config.time_per_step, 'k:', 'LineWidth', 1);
    end
    hold off;
end

function metrics = calculate_performance_metrics(t, x, params, config)
    %% 计算性能指标
    
    % 基本量
    omega_m = x(:, 2);
    I_m = x(:, 3);
    voltage_profile = arrayfun(@(time) get_voltage_at_time(time, config.voltages, config.time_per_step), t);
    
    % 计算各电压段的性能
    num_voltages = length(config.voltages);
    metrics.voltage_levels = config.voltages;
    metrics.steady_state_speed = zeros(num_voltages, 1); 
    metrics.steady_state_current = zeros(num_voltages, 1);
    metrics.power_consumption = zeros(num_voltages, 1);
    metrics.efficiency = zeros(num_voltages, 1);
    
    for i = 1:num_voltages
        % 找到对应时间段
        t_start = (i-1) * config.time_per_step;
        t_end = i * config.time_per_step;
        
        % 取稳态值（最后20%的时间）
        steady_start = t_start + 0.8 * config.time_per_step;
        steady_indices = (t >= steady_start) & (t <= t_end);
        
        if sum(steady_indices) > 0
            metrics.steady_state_speed(i) = mean(omega_m(steady_indices));
            metrics.steady_state_current(i) = mean(I_m(steady_indices));
            metrics.power_consumption(i) = config.voltages(i) * metrics.steady_state_current(i);
            
            % 效率计算（机械功率/电功率）
            mechanical_power = params.motor.kt * metrics.steady_state_current(i) * metrics.steady_state_speed(i);
            electrical_power = metrics.power_consumption(i);
            metrics.efficiency(i) = mechanical_power / electrical_power * 100;
        end
    end
    
    % 总体指标
    metrics.max_current = max(I_m);
    metrics.total_energy = trapz(t, voltage_profile .* I_m);
    metrics.avg_efficiency = mean(metrics.efficiency(~isnan(metrics.efficiency)));
end

function display_performance_summary(metrics)
    %% 显示性能总结
    
    fprintf('\n=== 系统性能总结 ===\n');
    fprintf('最大电流: %.3f A\n', metrics.max_current);
    fprintf('总能耗: %.3f J\n', metrics.total_energy);
    fprintf('平均效率: %.1f%%\n', metrics.avg_efficiency);
    
    fprintf('\n电压级别性能:\n');
    fprintf('电压(V)\t转速(rpm)\t电流(mA)\t功率(W)\t效率(%%)\n');
    fprintf('----------------------------------------------------\n');
    
    for i = 1:length(metrics.voltage_levels)
        rpm = metrics.steady_state_speed(i) * 60 / (2 * pi * 699.55);
        current_mA = metrics.steady_state_current(i) * 1000;
        
        fprintf('%.1f\t%.1f\t\t%.1f\t\t%.3f\t%.1f\n', ...
                metrics.voltage_levels(i), rpm, current_mA, ...
                metrics.power_consumption(i), metrics.efficiency(i));
    end
end