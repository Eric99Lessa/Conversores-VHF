function [D_bot, D_top, debug_obs, debug_deriv, debug_ctrl] = fcn(Vin_m, Vout_m, IL_m, converter_mode, Vout_ref, Ts)
%#codegen
    persistent cont ...
        L C RL VD G_load Idc_load CP_load ...
        RL_min RL_max VD_min VD_max...
        y_pred y_dot_pred ...
        p11 p12 p22 ... 
        R_meas Q Q_dot ...
        gamma_RL gamma_VD ...
        theta_load P_load lambda_load ...
        RL_dot VD_dot alpha_RL alpha_VD lambda_RL_VD ...
        D_bot_last D_top_last ...
        int_eVout initialized

    %% Fixed dimensions
    n_phases = 3;
    n_meas = n_phases + 2;
    n_est_val = n_meas + 1;
    n_est_der = n_phases + 1;

    %% Initialization
    if isempty(initialized)
        cont = 0;

        %% 
        D_bot_last = zeros(n_phases, 1);
        D_top_last = zeros(n_phases, 1);

        %% Integral term
        int_eVout = 0.0;

        %% Parameters
        L_nom   = 80/1000;
        C_nom   = 12/1000;
        L = L_nom*ones(n_phases, 1);
        RL = 10e-3*ones(n_phases, 1);
        VD  = 0.8*ones(n_phases, 1);
        C = C_nom;
        G_load = 0;
        Idc_load = 0;
        CP_load = 0;

        theta_load = [G_load; Idc_load; CP_load];
        lambda_load = 1 - Ts/0.3;
        P_load = eye(3);

        %
        RL_min = 1e-3;
        RL_max = 1e-1;
        VD_min = 0.7;
        VD_max = 1.8;
        
        %% 
        gamma_RL = 0.001;
        gamma_VD = 0.01;
        RL_dot = zeros(n_phases, 1);
        VD_dot = zeros(n_phases, 1);
        alpha_RL = 0.05;
        alpha_VD = 0.05;
        lambda_RL_VD = 1 - Ts/0.5;
        
        %% 
        y_pred = zeros(n_est_val, 1);
        y_pred(1:3) = IL_m;
        y_pred(4) = Vout_m;
        y_pred(5) = Vin_m;
        
        y_dot_pred = zeros(n_est_der, 1);
        p11 = ones(n_est_val, 1);
        p12 = zeros(n_est_der, 1);
        p22 = ones(n_est_der, 1);

        q_IL    = 1e-4;
        q_dIL   = 1e-1;   % main knob: derivative tracking speed
        q_Vout  = 1e-4;
        q_dVout = 1e-1;
        q_Vin   = 1e-4;
        q_Iload = 1e-2;
        Q = [q_IL*ones(n_phases, 1); q_Vout; q_Vin; q_Iload];
        Q_dot = [q_dIL*ones(n_phases, 1); q_dVout];
        
        % Measurement noise
        sigma_iL     = 0.1;
        sigma_Vout   = 0.1;
        sigma_Vin    = 0.1;
        sigma_Iload  = 1.0;   % ← PRIMARY SMOOTHING KNOB for Iload
                               %   larger  → smoother Iload, slower to react
                               %   smaller → Iload tracks node equation tightly
        
        R_meas = [
            sigma_iL^2   * ones(n_phases, 1);
            sigma_Vout^2;
            sigma_Vin^2;
            sigma_Iload^2                      % pseudo-measurement noise
        ];

        initialized = true;
    end

    %% Measurements
    idx_IL = 1:n_phases;
    idx_Vout = 4;
    idx_Vin = 5;
    idx_I_load = 6;
    y = zeros(n_est_val, 1);
    y(idx_IL) = IL_m(idx_IL);
    y(idx_Vout) = Vout_m;
    y(idx_Vin) = Vin_m;

    %% Update
    IL = zeros(n_phases, 1);
    IL_dot = zeros(n_phases, 1);
    for i = 1:n_phases
        [IL(i), IL_dot(i), p11(i), p12(i), p22(i)] = update_scalar_kf(y_pred(i), y_dot_pred(i), p11(i), p12(i), p22(i), y(i), R_meas(i));
    end

    [Vout, Vout_dot, p11(idx_Vout), p12(idx_Vout), p22(idx_Vout)] = update_scalar_kf( ...
        y_pred(idx_Vout), y_dot_pred(idx_Vout), p11(idx_Vout), p12(idx_Vout), p22(idx_Vout), y(idx_Vout), R_meas(idx_Vout));

    [Vin, p11(idx_Vin)] = update_scalar_kf_val(y_pred(idx_Vin), p11(idx_Vin), y(idx_Vin), R_meas(idx_Vin));

    if converter_mode == 0
        y(idx_I_load) = sum(IL.*(1 - D_bot_last)) - C*Vout_dot;
    else
        y(idx_I_load) = sum(IL) - C*Vout_dot;
    end
    [I_load, p11(idx_I_load)] = update_scalar_kf_val(y_pred(idx_I_load), p11(idx_I_load), y(idx_I_load), R_meas(idx_I_load));

    %% Power equations
    Pin = Vin*sum(IL); % Input power
    Pout = Vout*I_load; % Output power
    PR = sum(RL.*IL.*IL); % Resistive losses
    if converter_mode == 0
        PD = sum(VD.*(1 - D_bot_last).*IL); % Diode losses
    else
        PD = sum(VD.*(1 - D_top_last).*IL); % Diode losses
    end
    PL = sum(L.*IL.*IL_dot); % Inductive loading
    PC = C*Vout*Vout_dot; % Capacitive loading

    %% Derivatives by model
    if converter_mode == 0
        IL_dot_model = (Vin - RL.*IL - (Vout + VD).*(1 - D_bot_last))./L;
        Vout_dot_model = (sum(IL.*(1 - D_bot_last)) - I_load)/C;
    else
        IL_dot_model = ((Vin + VD).*D_top_last - RL.*IL - Vout - VD)./L;
        Vout_dot_model = (sum(IL) - I_load)/C;
    end
    
    %% MRAC to better estimate parameters
    % power residual
    res_P = Pin - Pout - PR - PD - PL - PC;

    % current residual
    y_IL = Vin - Vout*(1 - D_bot_last) - L.*IL_dot;
    y_IL_hat = RL.*IL + VD.*(1 - D_bot_last);
    res_IL = y_IL - y_IL_hat;

    % Update
    w_P = 0.1;
    w_IL = 1 - w_P;
    RL = RL + Ts*alpha_RL*RL_dot;
    VD = VD + Ts*alpha_VD*VD_dot;
    RL_dot = gamma_RL*(w_IL*res_IL.*IL + w_P*res_P.*IL.^2);
    VD_dot = gamma_VD*(w_IL*res_IL.*(1 - D_bot_last) + w_P*res_P.*(1 - D_bot_last).*IL);
    RL = RL + Ts*RL_dot;
    VD = VD + Ts*VD_dot;

    RL = min(max(RL, RL_min), RL_max);
    VD = min(max(VD, VD_min), VD_max);
    
    %% RLS to load estimation
    % Regressor vector
    phi_load = [Vout; 1; 1/(Vout + 0.001)];    % [3x1]
    
    % Prediction
    y_I_load_hat = theta_load' * phi_load;
    
    % Residual
    res_I_load = I_load - y_I_load_hat;
    
    if abs(res_I_load) > 0.1
        % RLS gain
        K_load = (P_load * phi_load) / (lambda_load + phi_load' * P_load * phi_load);  % [3x1]
        
        % Parameter update
        theta_load = theta_load + K_load * res_I_load;
        theta_load = max(theta_load, 0);
    
        % Covariance update
        P_load = (P_load - K_load * phi_load' * P_load) / lambda_load;
    end
    
    % Unpack estimates
    G_load  = min(max(theta_load(1), 0), 2);
    Idc_load = min(max(theta_load(2), 0), 160);
    CP_load  = min(max(theta_load(3), 0), 15000);

    %% Inductor current equilibrium
    I_load_eq = G_load*Vout_ref + Idc_load + CP_load/(Vout_ref + 0.001);
    I_load_eq = max(I_load_eq, I_load);
    a = sum(RL./(VD + Vout_ref));
    b = -sum(1./(VD + Vout_ref))*Vin;
    c = I_load_eq;

    if b^2 - 4*a*c > 0
        IL_eq = (-b - sqrt(b^2 - 4*a*c))/(2*a);
    elseif b^2 - 4*a*I_load > 0
        IL_eq = (-b - sqrt(b^2 - 4*a*I_load))/(2*a);
    else
        IL_eq = 0;
    end

    %% Voltage error and integral
    e_Vout = Vout_ref - Vout;

    if abs(e_Vout) < 0.5*max(Vout_ref, 1.0)
        int_eVout = int_eVout + e_Vout * Ts;
    end

    %% Control laws
    D_bot = zeros(n_phases, 1);
    D_top = zeros(n_phases, 1);
    if converter_mode == 0
        D_bot(1:n_phases) = 0.5*ones(n_phases, 1) + 0.01*sin(2*6.28*cont*Ts);
        cont = cont + 1;
    else
        D_top(1:n_phases) = 0.5*ones(n_phases, 1);
    end
    u = [D_bot; D_top];
    D_bot_last = D_bot;
    D_top_last = D_top;

    %% Prediction
    for i = 1:n_phases
        [y_pred(i), y_dot_pred(i), p11(i), p12(i), p22(i)] = predict_scalar_kf(IL(i), IL_dot(i), p11(i), p12(i), p22(i), Q(i), Q_dot(i), Ts);
    end
    [y_pred(idx_Vout), y_dot_pred(idx_Vout), p11(idx_Vout), p12(idx_Vout), p22(idx_Vout)] = ...
        predict_scalar_kf(Vout, Vout_dot, p11(idx_Vout), p12(idx_Vout), p22(idx_Vout), Q(idx_Vout), Q_dot(idx_Vout), Ts);

    [y_pred(idx_Vin), p11(idx_Vin)] = predict_scalar_kf_val(Vin, p11(idx_Vin), Q(idx_Vin));
    [y_pred(idx_I_load), p11(idx_I_load)] = predict_scalar_kf_val(I_load, p11(idx_I_load), Q(idx_I_load));

    %% Debug variables
    debug_obs = zeros(40, 1);
    debug_obs(idx_IL) = IL;
    debug_obs(idx_Vout) = Vout;
    debug_obs(idx_Vin) = Vin;
    debug_obs(idx_I_load) = I_load;
    debug_obs((1:n_est_val) + 6) = y_pred;
    debug_obs(13:15) = L;
    debug_obs(16) = C;
    debug_obs(17:19) = RL;
    debug_obs(20:22) = VD;
    debug_obs(23) = G_load;
    debug_obs(24) = Idc_load;
    debug_obs(25) = CP_load;
    debug_obs(26) = Pin;
    debug_obs(27) = Pout;
    debug_obs(28) = PR;
    debug_obs(29) = PD;
    debug_obs(30) = PL;
    debug_obs(31) = PC;
    debug_obs(32) = res_P;
    debug_obs(33:35) = res_IL;
    debug_obs(36) = res_I_load;

    debug_deriv = zeros(15, 1);
    debug_deriv(1:3) = IL_dot;
    debug_deriv(4) = Vout_dot; 
    debug_deriv(5:7) = IL_dot_model;
    debug_deriv(8) = Vout_dot_model;

    debug_ctrl = zeros(15, 1);
    debug_ctrl(1:6) = u;
    debug_ctrl(7) = IL_eq;
    debug_ctrl(8) = int_eVout;
end

function [x_val, x_dot, p11, p12, p22] = update_scalar_kf(x_pred, x_dot_pred, p11, p12, p22, z, r)
    innov = z - x_pred;

    S = p11 + r;       % innovation covariance
    k1 = p11/S;        % Kalman gain
    k2 = p12/S;        % Kalman gain 

    x_val = x_pred + k1*innov;
    x_dot = x_dot_pred + k2*innov;

    % covariance update
    p22 = p22 - k2*p12;
    p12 = p12 - k1*p12;
    p11 = p11 - k1*p11;
end

function [x_val_p, x_dot_p, p11_p, p12_p, p22_p] = predict_scalar_kf(x_val, x_dot, p11, p12, p22, q_val, q_dot, Ts)
    % ── State propagation f(x) ────────────────────────────────────────────
    x_dot_p = x_dot;
    x_val_p = x_val + Ts*x_dot;

    % ── Covariance prediction ─────────────────────────────────────────────
    p11_p = p11 + 2*Ts*p12 + p22*Ts*Ts + q_val;
    p12_p = p12 + Ts*p22;
    p22_p = p22 + q_dot;
end

function [x_val, p] = update_scalar_kf_val(x_pred, p, z, r)
    S = p + r;       % innovation covariance
    k = p/S;        % Kalman gain

    x_val = x_pred + k*(z - x_pred);

    p = p - k*p;    % covariance update
end

function [x_val_p, p] = predict_scalar_kf_val(x_val, p, q_val)
    x_val_p = x_val;
    % ── Covariance prediction ─────────────────────────────────────────────
    p = p + q_val;
end