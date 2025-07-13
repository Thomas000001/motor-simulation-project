%----------Energy Consumption of Geared DC Motors in Dynamic Applications: Comparing Modeling Approaches
%60rpm
function ac_dc_current_analysis()
    % clear; clc; 
    % close all;


    %1.參數定義

    %----連桿機構與負載常數----
%**************************記得確認彈簧常數、齒輪比、電阻***************************
    params.r = 6;
    params.l = 22;
    % params.k_spring = 0.1883323478; %彈簧常數(N/mm)0.3x5x10
    params.k_spring = 0.1031979802*1.5; %彈簧常數(N/mm)0.2x3x10
    % params.k_spring = 0.128421874; %彈簧常數(N/mm)
    params.dist_to_spring = 5.566;
    params.max_compression = 4.5; %彈簧最大壓縮量

    %----馬達電器-機械參數----
    % params.R_m = 38; %60rpm電阻
    params.R_m = 38*1.1; %300rpm電阻
    params.L_m = 0.015e-3;
    params.kt = 0.194*600;
    params.kb = 0.0532/250; %(V/(rad/s))
    params.J_m = 1e-7;
    % params.n_gear = 699.55; %齒輪比：60rpm
    params.n_gear = 136.02; %齒輪比：300rpm
    params.efficiency = 0.8; % ***新增*** 減速機效率 (典型值約 0.7-0.9，請根據規格書修改)
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
    % time_per_step = 7.5;     % low loading下每個電壓階躍持續的時間 (s)
    time_per_step = 12.5;     % high loading下每個電壓階躍持續的時間 (s)
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
        
    fprintf('仿真完成！開始ACDC分析...\n');
    
    %% 3. 後處理：AC/DC 成分分析 (含穩健的卡死檢測)
    % ---------------------------------------------------------------------
    num_voltages = length(voltages_to_test);
    % *** 修改：初始化為 NaN，表示數據尚未計算 ***
    I_dc_results = NaN(num_voltages, 1);
    I_ac_results = NaN(num_voltages, 1);
    
    for i = 1:num_voltages
        start_time = (i-1) * time_per_step;
        end_time = i * time_per_step;
        analysis_start_time = start_time + time_per_step / 2;
        segment_indices = find(t >= analysis_start_time & t < end_time); 
        %找出每個驅動電壓的持續時間
        
        if length(segment_indices) < 12, continue; end

        t_stable = t(segment_indices);
        dtheta_m_stable = x(segment_indices, 2);
        I_m_stable = x(segment_indices, 3);
        
        % *** 新增：雙條件卡死檢測邏輯 ***
        % ------------------------------------------------------------------
        stall_speed_threshold = 1e-2; % rad/s
        stall_current_std_threshold = 1e-2; % A, 電流標準差閾值

        current_derivative = diff(I_m_stable) ./ diff(t_stable);
        
        is_speed_stalled = mean(abs(dtheta_m_stable)) < stall_speed_threshold;
        % is_current_stalled = std(I_m_stable) < stall_current_std_threshold;
        is_current_stalled = mean(abs(current_derivative)) < stall_current_std_threshold;

        if is_speed_stalled && is_current_stalled
            fprintf('信息：在電壓 %.1fV 時，檢測到馬達卡死 (低速且電流穩定)。\n', voltages_to_test(i));
            % 當卡死時，不計算 AC/DC 值，保持其為 NaN，直接進入下一次迴圈
            continue;
        end
        % ------------------------------------------------------------------
        
        % 如果未卡死，則繼續進行週期分析
        revolutions = floor(x(segment_indices, 1) / (2*pi));
        %   x(segment_indices,1)取出在某個電壓下紀錄的馬達角度轉動值
        %   除上2pi是為了計算角度對應的圈數，除上n_gear是因為馬達還有gearbox
        %   用floor計算四捨五入，是為了將尚未到達一圈的值視為同一圈，例如0.1,0.5,0.9等都將視為第0圈，1.1,1.2...1.9都視為第一圈，以此類推
        %   最終revolutions的輸出應該會為由>=0的整數所構成
        cycle_boundaries = find(diff(revolutions) > 0);
        %   通過diff函數來計算revolutions數組內元素變化的次數即為旋轉圈數

        % *** 修改：檢查是否存在至少三個完整週期 (需要四個邊界點) ***
        if length(cycle_boundaries) < 2
            fprintf('警告：在電壓 %.1fV 時，未能找到足夠的週期進行多週期平均分析。\n', voltages_to_test(i));
            continue;
        end

        % *** 修改：選取倒數三個完整週期的索引 ***
        start_idx = cycle_boundaries(end-1);
        end_idx = cycle_boundaries(end);
        multi_cycle_indices = start_idx:end_idx;

        t_multi_cycle = t_stable(multi_cycle_indices);
        %   將電流單位轉為mA進行後續的計算
        I_multi_cycle = I_m_stable(multi_cycle_indices) * 1000;
        
        cycle_duration = t_multi_cycle(end) - t_multi_cycle(1);
        if cycle_duration < 1e-6, continue; end

        % total_charge = trapz(t_cycle, I_cycle);
        % I_dc = total_charge / cycle_duration;
        I_dc = mean(I_multi_cycle);
        I_dc_results(i) = I_dc;

        I_ac_parts = I_multi_cycle(I_multi_cycle > I_dc);
        if ~isempty(I_ac_parts)
            I_ac_results(i) = mean(I_ac_parts);
            % I_ac_results(i) = sqrt(mean((I_ac_parts - I_dc).^2));
        else
            I_ac_results(i) = I_dc_results(i);
        end
    end
    fprintf('AC/DC 成分分析完成！\n');
    
    %% 4. 輸出與繪圖
    % ---------------------------------------------------------------------
    num_metrics = num_voltages - 1;
    
    output_voltages = NaN(num_metrics, 1);
    output_DC = NaN(num_metrics, 1);
    output_AC = NaN(num_metrics, 1);
    output_sensitivity = NaN(num_metrics, 1);

    for i = 1:num_metrics
        %   獲取前一個電壓和當前電壓
        V_current = voltages_to_test(i);
        V_next = voltages_to_test(i+1);
        
        DC_V = I_dc_results(i);
        %   將電流單位轉換為mA來計算
        DC_V_next = I_dc_results(i+1);
        AC_V_next = I_ac_results(i+1);
        
        % 如果任一數據點因卡死或分析失敗而為 NaN，則跳過
        if isnan(DC_V) || isnan(DC_V_next), continue; end
        
        delta_dc = DC_V - DC_V_next;
        
        if abs(delta_dc) > 1e-6
            sensitivity = AC_V_next / abs(delta_dc);
            
            output_voltages(i) = V_next;
            output_DC(i) = DC_V;
            output_AC(i) = AC_V_next;
            output_sensitivity(i) = sensitivity;
        end
    end

    % --- 輸出結果匯總表 ---
    fprintf('\n======================== 敏感度指標分析結果 ========================\n');
    fprintf('電壓 (V) | AC[V_next] (A) | DC[V] (A) | DC[V_next] (A) | 指標 (AC/ΔDC)\n');
    fprintf('----------------------------------------------------------------------\n');
    for i = 1:num_metrics
         V = voltages_to_test(i);
         if ~isnan(output_sensitivity(i))
            fprintf('  %.1f    |    %.5f     |  %.5f  |   %.5f    |    %.2f\n', ...
                V, output_AC(i), output_DC(i), I_dc_results(i+1), output_sensitivity(i));
         else
            fprintf('  %.1f    |             (數據不足或卡死無法計算)             |\n', V);
         end
    end
    fprintf('======================================================================\n\n');

    % --- 繪製結果圖 ---
    figure('Name', '電流敏感度指標分析 V3 (穩健卡死檢測)', 'Position', [100, 100, 900, 600]);
    
    valid_indices = ~isnan(output_sensitivity);
    
    plot(output_voltages(valid_indices), output_sensitivity(valid_indices), ...
        'r-o', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', 'r');
    
    title('電流敏感度指標 vs. 輸入電壓');
    xlabel('輸入電壓 (V)');
    ylabel('敏感度指標 (AC_{V_{next}} / \DeltaDC)');
    grid on;
    
    % ax = gca;
    % ax.XDir = 'reverse';

    sgtitle('馬達電流敏感度隨驅動電壓的變化分析 (V3)');
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
    % T_net = T_motor_gen - T_friction + T_crank;
    T_net = T_motor_gen - T_friction + T_load;
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