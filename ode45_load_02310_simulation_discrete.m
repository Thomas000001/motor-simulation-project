%----------Energy Consumption of Geared DC Motors in Dynamic Applications: Comparing Modeling Approaches
%60rpm
function ode45_load_02310_simulation_discrete()
    clear; clc; 
    % close all;


    %1.參數定義

    %----連桿機構與負載常數----
    params.r = 6;
    params.l = 22;
    % params.k_spring = 0.1883323478; %彈簧常數(N/mm)0.3x5x10
    params.k_spring = 0.1031979802*1.5; %彈簧常數(N/mm)0.2x3x10
    % params.k_spring = 0.128421874; %彈簧常數(N/mm)
    params.dist_to_spring = 5.566;
    params.max_compression = 4.5; %彈簧最大壓縮量

    %----馬達電器-機械參數----
    params.R_m = 38; %60rpm電阻
    % params.R_m = 38*1.1; %300rpm電阻
    params.L_m = 0.015e-3;
    params.kt = 0.194*600;
    params.kb = 0.0532/250; %(V/(rad/s))
    params.J_m = 1e-7;
    params.n_gear = 699.55; %彈簧常數：60rpm
    % params.n_gear = 136.02; %彈簧常數：60rpm
    params.J_gear = 5e-4/3;
    %馬達的內部黏性阻尼
    params.v_m = 1.25012e-3;
    params.v_track = 0.2;   %連桿材料的黏性摩擦係數
    params.Fc_track = 0.1;       % 軌道庫倫摩擦力 (N) (示例值)

    %馬達啟動最大靜摩擦力矩
    params.Ts_motor = 0.00485;

    %熱力學參數
    params.alpha = 0.0039; %銅的電阻溫度係數
    params.T_ref = 25;


    %2.仿真設定
    voltages_to_test = 5.0:-0.2:0.6; % 從 5.0V 到 0.6V，間隔 0.2V
    num_voltages = length(voltages_to_test);
    time_per_step = 12.5;     % 每個電壓階躍持續的時間 (s)
    total_duration = time_per_step * num_voltages;  
    tspan = [0, total_duration];    % 設置總共的仿真時間範圍

    % 使用 cell array 來存儲每次仿真的結果
    % results = cell(num_voltages, 1);

    x0 = [0; 0; 0];  % 系統的初始條件 [theta_m; dtheta_m; I_m]

    %% 3. 調用 ODE45 進行單次連續仿真
    % ---------------------------------------------------------------------
    fprintf('開始進行連續階躍降壓仿真...\n');
    options = odeset('RelTol', 1e-3, 'AbsTol', [1e-5 1e-5 1e-4]);
    [t, x] = ode45(@(t,x) odefun_motor_spring_load(t, x, ...
        get_voltage_at_time(t, voltages_to_test, time_per_step), ...
        params), tspan, x0, options);
        
    fprintf('仿真完成！\n');
    %% 4. 繪製最終結果圖
    % ---------------------------------------------------------------------
    figure('Name', '連續階躍降壓動態響應', 'Position', [100, 100, 900, 700]);
    
    % --- 上半部子圖：馬達角速度 vs. 時間 ---
    subplot(3, 1, 1);
    plot(t, x(:, 1) / (2*pi*params.n_gear), 'b-', 'LineWidth', 1.5);
    title('馬達旋轉圈數的連續響應');
    xlabel('時間 (s)');
    ylabel('角速度 \theta_m (rev)');
    
    subplot(3, 1, 2);
    plot(t, x(:, 2) / (2*pi*params.n_gear), 'b-', 'LineWidth', 1.5);
    title('馬達角速度的連續響應');
    xlabel('時間 (s)');
    ylabel('角速度 \omega_m (rev/s)');
    grid on;
    % 為了看得更清楚，可以在電壓切換的時刻畫上垂直線
    hold on;
    for i = 1:(length(voltages_to_test)-1)
        plot([i*time_per_step, i*time_per_step], ylim, 'k:');
    end
    hold off;

    % --- 下半部子圖：馬達電流 vs. 時間 ---
    subplot(3, 1, 3);
    plot(t, x(:, 3)*1000, 'r-', 'LineWidth', 1.5);
    title('馬達電流的連續響應');
    xlabel('時間 (s)');
    ylabel('電流 I_m (A)');
    grid on;
    % 為了看得更清楚，可以在電壓切換的時刻畫上垂直線
    hold on;
    for i = 1:(length(voltages_to_test)-1)
        plot([i*time_per_step, i*time_per_step], ylim, 'k:');
    end
    hold off;
    
    sgtitle('馬達-連桿-彈簧負載系統的連續階躍降壓響應分析');
end

% =========================================================================
% 輔助函數：根據時間獲取對應的電壓值
% =========================================================================
function V_current = get_voltage_at_time(t, voltages, step_duration)
    % 計算當前時間 t 屬於第幾個區間
    % floor(t / step_duration) 會得到 0, 1, 2, ...
    % +1 後得到索引 1, 2, 3, ...
    idx = floor(t / step_duration) + 1;
    
    % 防止時間超出範圍導致索引越界
    if idx > length(voltages)
        idx = length(voltages);
    end
    
    V_current = voltages(idx);
end

%狀態空間函數(有負載)
function dxdt = odefun_motor_spring_load(t, x, V_input, params)
    %1.解包狀態變量和參數
    theta_m = x(1);
    dtheta_m = x(2);    %馬達角速度
    I_m = x(3);         %馬達電流

    %馬達參數定義
    kt = params.kt; kb = params.kb; R_m = params.R_m; L_m = params.L_m;
    J_motor = params.J_m; J_gear = params.J_gear; n_gear = params.n_gear;
    v_motor = params.v_m;
    Ts_motor = params.Ts_motor;
    % dist_to_spring = params.dist_to_spring;
    r = params.r; l = params.l; k_spring = params.k_spring; v_track = params.v_track;
    Fc_track = params.Fc_track;
    dist_to_spring = params.dist_to_spring; max_compression = params.max_compression;


    %dx1dt = d(dtheta)/dt %角加速度

    %----計算由彈簧產生的負載力矩 T_load----
    %a.計算滑塊位置和前進距離
    theta_crank = theta_m / n_gear;
    x_slider = r * cos(theta_crank) + sqrt(l^2 - (r * sin(theta_crank))^2);
    x_min = l - r;
    distance = x_slider - x_min;

    %b.計算彈簧壓縮量和彈簧力
    compression = distance - dist_to_spring;
    F_spring = 0;   %默認彈簧力為0
    if compression > 0 
        if compression > max_compression
            F_spring = k_spring * max_compression;
        else
            F_spring = k_spring * compression;
        end
    end

    % 彈簧力方向與前進方向相反
    F_spring = -F_spring;

    % c. 計算雅可比 (Jacobian)，即滑塊速度與曲柄角速度的關係
    % v_slider = J * omega_crank
    term_in_sqrt = l^2 - (r * sin(theta_crank))^2;
    if term_in_sqrt < 0; term_in_sqrt = 0; end % 避免數值錯誤
    jacobian = -r * sin(theta_crank) - (r^2 * sin(theta_crank) * cos(theta_crank)) / sqrt(term_in_sqrt);
    
    %計算滑塊當前速度
    dtheta_crank = dtheta_m / n_gear;
    v_slider = dtheta_crank * jacobian;
    F_track_friction = -(Fc_track * sign(v_slider) + v_track * v_slider);

    % d. 將彈簧力轉換為作用在曲柄上的力矩，再折算到馬達軸
    % Power_load = Power_motor => F_spring * v_slider = T_crank * omega_crank
    % T_crank = (F_spring + F_track_friction) * jacobian;   %加上機構造成的摩擦力
    T_crank = (F_spring) * jacobian;
    T_load = T_crank / n_gear; % 折算到馬達軸的負載力矩


    %馬達啟動時要克服的啟動力矩
    epsilon = 1e-6; %速度threshold
    
    %馬達產生的力矩
    T_motor_gen = kt * I_m;

    %最大靜摩擦判斷
    if abs(dtheta_m) < epsilon
        %靜止狀態
        if abs(T_motor_gen) < Ts_motor
            %驅動馬達產生的力矩不足以克服最大靜摩擦力
            T_friction = T_motor_gen;
        else
            %擺脫最大靜摩擦力,轉為動摩擦力
            T_friction = Ts_motor * 0.9;
        end
    else
        %運動狀態
        T_friction = Ts_motor * 0.8 + v_motor * dtheta_m;

    end

    %dx1dt = d(theta_m)/dt
    dx1dt = dtheta_m;

    %dx2dt = d(dtheta_m)/dt
    
    %總慣量
    J_total = J_motor + J_gear;

    %淨力矩=產生力矩-摩擦力矩
    T_net = T_motor_gen - T_friction + T_crank; 
    %此處加上T_crank而不是減去T_crank是由於前述已經將彈簧力施加一個反向符號

    dx2dt = T_net / J_total;
    
    % R_current = R_m * (1 + (0.008 * t)*0.0039);

    %2.計算狀態的導數
    %dx2/dt = d(I_m)/dt
    V_emf = kb * dtheta_m;
    dx3dt = (1/L_m) * (V_input - R_m * I_m - V_emf);

    %3.return
    dxdt = [dx1dt; dx2dt; dx3dt];
end