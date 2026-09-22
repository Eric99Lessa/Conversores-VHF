function [D_bot, D_top, debug_obs, debug_deriv, debug_ctrl] = fcn(Vin_m, Vout_m, IL_m, converter_mode, Vout_ref, Ts)
%#codegen
    persistent cont ...
        L C RL VD G_load Idc_load CP_load ...
        x_pred F H P_pred Q R_meas ...
        idx_IL idx_Vout idx_Vin idx_dIL idx_dVout idx_Iload ...
        gamma_RL gamma_VD gamma_G_load gamma_Idc_load gamma_CP_load ...
        D_bot_last D_top_last ...
        int_eVout initialized

    %% Fixed dimensions
    n_phases = 3;
    n_meas = n_phases + 2; % one pseudo measurement
    n_states = 2*n_meas - 1 + 1; % measurements and its derivatives and load current

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

        %% 
        idx_IL    = 1          : n_phases;
        idx_Vout = n_phases + 1;
        idx_Vin = n_phases + 2;
        idx_dIL   = (n_phases+3):(2*n_phases + 2);
        idx_dVout = 2*n_phases + 3;
        % idx_dVin  = 2*n_phases + 4;
        idx_Iload = 2*n_phases + 4;

        %% 
        gamma_RL = 0.01;
        gamma_VD = 0.05;
        gamma_G_load = 0.1; 
        gamma_Idc_load = 1;
        gamma_CP_load = 0.1;

        %% EKF Matrixes
        H = zeros(n_meas + 1, n_states); % Add one pseudo measurement
        % Physical measurements
        for i = 1:n_phases
            H(i, idx_IL(i)) = 1;           % measure IL_i
        end
        H(n_phases+1, idx_Vout) = 1;       % measure Vout
        H(n_phases+2, idx_Vin)  = 1;       % measure Vin

        % ── Jacobian F = ∂f/∂x ───────────────────────────────────────────────

        F = eye(n_states);
        for i = 1:n_phases
            F(idx_IL(i),  idx_dIL(i)) = Ts;
        end
        F(idx_Vout, idx_dVout) = Ts;
        % F(idx_Vin,  idx_dVin)  = Ts;

        x_pred = zeros(n_states, 1);
        x_pred(idx_IL) = IL_m;
        x_pred(idx_Vout) = Vout_m;
        x_pred(idx_Vin) = Vin_m;
        P_pred = eye(n_states);

        % Process noise
        q_IL    = 1e-4;
        q_dIL   = 1e-1;   % main knob: derivative tracking speed
        q_Vout  = 1e-4;
        q_dVout = 1e-1;
        q_Vin   = 1e-4;
        % q_dVin  = 1e-1;
        q_Iload = 1e-2;   % random-walk noise on Iload; tune for tracking speed
        
        Q = diag([
            q_IL    * ones(n_phases, 1);
            q_Vout;
            q_Vin;
            q_dIL   * ones(n_phases, 1);
            q_dVout;
            q_Iload
        ]);

        % Measurement noise
        sigma_iL     = 0.1;
        sigma_Vout   = 0.1;
        sigma_Vin    = 0.1;
        sigma_Iload  = 1.0;   % ← PRIMARY SMOOTHING KNOB for Iload
                               %   larger  → smoother Iload, slower to react
                               %   smaller → Iload tracks node equation tightly
        
        R_meas = diag([
            sigma_iL^2   * ones(n_phases, 1);
            sigma_Vout^2;
            sigma_Vin^2;
            sigma_Iload^2                      % pseudo-measurement noise
        ]);

        initialized = true;
    end

    %% Measurement
    z = [IL_m; Vout_m; Vin_m; 0];

    %% Update
    H(n_meas + 1, 1:n_phases) = 1 - D_bot_last';
    H(n_phases+3, idx_dVout) = -C;         % ∂node/∂dVout
    H(n_phases+3, idx_Iload) = -1;         % ∂node/∂Iload
    [x_hat, P_hat] = ekf_update(x_pred, P_pred, z, H, R_meas, n_states);

    %% Extract states
    IL = x_hat(idx_IL);
    Vout = x_hat(idx_Vout);
    Vin = x_hat(idx_Vin);
    IL_dot = x_hat(idx_dIL);
    Vout_dot = x_hat(idx_dVout);
    I_load = x_hat(idx_Iload);

    %% Power equations
    Pin = Vin*sum(IL); % Input power
    Pout = Vout*I_load; % Output power
    PR = sum(RL.*IL.^2); % Resistive losses
    if converter_mode == 0
        PD = sum(VD.*(1 - D_bot_last).*IL); % Diode losses
    else
        PD = sum(VD.*(1 - D_top_last).*IL); % Diode losses
    end
    PL = sum(L.*IL.*IL_dot); % Inductive loading
    PC = C*Vout*Vout_dot; % Capacitive loading

    %% Derivatives by model
    IL_dot_model = (Vin - RL.*IL - (Vout + VD).*(1 - D_bot_last))./L;
    Vout_dot_model = (sum(IL.*(1 - D_bot_last)) - I_load)/C;

    %% MRAC to better estimate parameters
    % power residual
    res_P = Pin - Pout - PR - PD - PL - PC;

    % current residual
    y_IL = Vin - Vout*(1 - D_bot_last);
    y_IL_hat = L.*IL_dot + RL.*IL + VD.*(1 - D_bot_last);
    res_IL = y_IL - y_IL_hat;

    % load residual
    y_I_load_hat = G_load*Vout + Idc_load + CP_load/(Vout + 0.001);
    res_I_load = I_load - y_I_load_hat;

    % Update
    w_P = 1/50;
    RL = RL + Ts*gamma_RL*(res_IL.*IL + w_P*res_P.*IL.^2);
    VD = VD + Ts*gamma_VD*(res_IL.*(1 - D_bot_last) + w_P*res_P.*(1 - D_bot_last).*IL);
    G_load = G_load + Ts*gamma_G_load*(res_I_load*Vout);
    Idc_load = Idc_load + Ts*gamma_Idc_load*(res_I_load);
    CP_load = CP_load + Ts*gamma_CP_load*(res_I_load/(Vout + 0.001));

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
    [x_pred, P_pred] = ekf_predict(x_hat, P_hat, Q, F, Ts, ...
                                         idx_IL, idx_dIL, idx_Vout, idx_dVout);

    %% Debug variables
    debug_obs = zeros(40, 1);
    debug_obs(1:5) = x_pred(1:n_meas);
    debug_obs(6:10) = x_hat(1:n_meas);
    debug_obs(11:13) = IL;
    debug_obs(14) = Vout;
    debug_obs(15) = Vin;
    debug_obs(16) = I_load;
    debug_obs(17) = G_load;
    debug_obs(18) = Idc_load;
    debug_obs(19) = CP_load;
    debug_obs(20:22) = L;
    debug_obs(23) = C;
    debug_obs(24:26) = RL;
    debug_obs(27:29) = VD;
    debug_obs(30) = Pin;
    debug_obs(31) = Pout;
    debug_obs(32) = PR;
    debug_obs(33) = PD;
    debug_obs(34) = PL;
    debug_obs(35) = PC;
    debug_obs(36) = res_P;
    debug_obs(37:39) = res_IL;
    debug_obs(40) = res_I_load;

    debug_deriv = zeros(25, 1);
    debug_deriv(1:3) = IL_dot;
    debug_deriv(4) = Vout_dot;
    % debug_deriv(5) = Vin_dot;
    debug_deriv(6:8) = IL_dot_model;
    debug_deriv(9) = Vout_dot_model;

    debug_ctrl = zeros(25, 1);
    debug_ctrl(1:6) = u;
    debug_ctrl(7) = int_eVout;
end

function [x_upd, P_upd] = ekf_update(x_pred, P_pred, z, H, R, n_states)
    % EKF_UPDATE  Standard linear measurement update  (z = H·x is linear).
    
    %  z includes both real measurements AND the pseudo-measurement row,
    %  so Iload is softly pulled toward the node equation with weight
    %  determined by sigma_Iload (the last diagonal of R).

    y = z - H*x_pred;          % innovation (node row: 0 - H_node·x_pred)

    PHt = P_pred*H';
    S = H*PHt + R;     % innovation covariance
    K = PHt/S;         % Kalman gain

    x_upd = x_pred + K*y;

    % Joseph form for numerical stability
    KH = K*H;
    IKH = eye(n_states) - KH;
    P_upd = IKH*P_pred*IKH' + K*R*K';
    % P_upd = (eye(n_states) - K*H)*P_pred;
end

function [x_pred, P_pred] = ekf_predict(x, P, Q, F, Ts, ...
                                         idx_IL, idx_dIL, idx_Vout, idx_dVout)
    dIL   = x(idx_dIL);
    dVout = x(idx_dVout);

    % ── State propagation f(x) ────────────────────────────────────────────
    x_pred = x;
    x_pred(idx_IL)    = x_pred(idx_IL)    + Ts * dIL;
    x_pred(idx_Vout)  = x_pred(idx_Vout)  + Ts * dVout;

    % ── Covariance prediction ─────────────────────────────────────────────
    P_pred = F * P * F' + Q;
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