function [D_bot, D_top, debug_obs, debug_deriv, debug_ctrl] = fcn(Vin_m, Vout_m, IL_m, converter_mode, Vout_ref, Ts, n_phases)
%#codegen
    persistent L_nom C_nom ...
        x_pred Px_pred theta_pred Ptheta_pred Gamma_pred ...
        Qx R_meas R_theta ...
        lambda_x lambda_t ...
        D_bot_last D_top_last ...
        int_eVout initialized

    %% Fixed dimensions
    n_states = 5;
    n_meas = 5;
    n_theta = 3*3 + 4;

    %% Initialization
    if isempty(initialized)
        %% Initial state estimate
        x0 = zeros(n_states, 1);

        x0(1) = IL_m(1);
        x0(2) = IL_m(2);
        x0(3) = IL_m(3);
        x0(4) = Vout_m;
        x0(5) = Vin_m;

        x_pred = x0;

        %% Parameters
        L_nom   = 80/1000;
        C_nom   = 12/1000;
        RL = 10e-3*ones(3, 1);
        VD  = 0.8*ones(3, 1);
        G_load = 0;
        Idc_load = 0;
        CP_load = 0;
        
        theta_pred = zeros(n_theta, 1);
        theta_pred(1:3) = 1/L_nom;
        theta_pred(4:6) = RL/L_nom;
        theta_pred(7:9) = VD/L_nom;
        theta_pred(10) = 1/C_nom;
        theta_pred(11) = G_load/C_nom;
        theta_pred(12) = Idc_load/C_nom;
        theta_pred(13) = CP_load/C_nom;

        %% Initial covariance
        Px_pred = eye(n_states);
        Ptheta_pred = eye(n_theta);
        Gamma_pred = zeros(n_states, n_theta);

        %% Measurement noise covariance
        sigma_iL   = 0.1;
        sigma_Vout = 0.1;
        sigma_Vin  = 0.1;

        R_meas = diag([
            sigma_iL^2;
            sigma_iL^2;
            sigma_iL^2;
            sigma_Vout^2;
            sigma_Vin^2
        ]);

        R_theta = 0.01*eye(n_meas);

        Qx = diag([
            1e-2^2;     % iL1 model uncertainty
            1e-2^2;     % iL2 model uncertainty
            1e-2^2;     % iL3 model uncertainty
            1e-2^2;     % Vout model uncertainty
            1e-2^2     % Vin random-walk/filtering noise
        ]);

        lambda_x = exp(-Ts/0.01);
        lambda_t = exp(-Ts/0.5);

        %% 
        D_bot_last = zeros(3, 1);
        D_top_last = zeros(3, 1);

        %% Integral term
        int_eVout = 0.0;

        initialized = true;
    end

    %% Measurement vector at current sample
    y = [IL_m; Vout_m; Vin_m];

    %% update step
    [x_hat, theta_hat, Gamma_hat, Kx, Ktheta] = update_state_theta( ...
        y, x_pred, theta_pred, Px_pred, Ptheta_pred, Gamma_pred, R_meas, R_theta, n_meas);

    IL = x_hat(1:3);
    Vout = x_hat(4);
    Vin = x_hat(5);

    tol_L = 0.1;
    L_min = (1 - tol_L)*L_nom;
    L_max = (1 + tol_L)*L_nom;
    theta_hat(1:3) = min(max(theta_hat(1:3), 1/L_max), 1/L_min);
    L = 1./theta_hat(1:3);
    theta_hat(4:6) = min(max(theta_hat(4:6).*L, 0.0001), 0.1)./L;
    theta_hat(7:9) = min(max(theta_hat(7:9).*L, 0.7), 2)./L;

    tol_C = 0.1;
    C_min = (1 - tol_C)*C_nom;
    C_max = (1 + tol_C)*C_nom;
    theta_hat(10) = min(max(theta_hat(10), 1/C_max), 1/C_min);
    C = 1/theta_hat(10);
    theta_hat(11) = min(max(theta_hat(11)*C, 0), 2)/C;
    theta_hat(12) = min(max(theta_hat(12)*C, 0), 160)/C;
    theta_hat(13) = min(max(theta_hat(13)*C, 0), 15000)/C;
    
    RL = theta_hat(4:6).*L;
    VD = theta_hat(7:9).*L;    
    G_load = theta_hat(11)*C;
    Idc_load = theta_hat(12)*C;
    CP_load = theta_hat(13)*C;

    I_load = G_load*Vout + Idc_load + CP_load/(Vout + 0.001);

    %% Derivatives
    % By model
    IL_dot_model = zeros(3, 1);
    if converter_mode == 0
        ID_sum = 0.0;
        for i = 1:n_phases
            ID = (1.0 - D_bot_last(i))*IL(i);
            IL_dot_model(i) = (Vin - RL(i)*IL(i) - (1.0 - D_bot_last(i))*(Vout + VD(i)))/L(i);
            ID_sum = ID_sum + ID;
        end
        Vout_dot_model = (ID_sum - I_load)/C;
    else
        for i = 1:n_phases
            IL_dot_model(i) = -(VD(i) + Vout + RL(i)*IL(i) - (Vin + VD(i))*D_top_last(i))/L(i);
        end
        Vout_dot_model = (sum(IL) - I_load)/C;
    end
    IL_dot = IL_dot_model;
    Vout_dot = Vout_dot_model;
    
    %% Voltage error and integral
    e_Vout = Vout_ref - Vout;

    if abs(e_Vout) < 0.5*max(Vout_ref, 1.0)
        int_eVout = int_eVout + e_Vout * Ts;
    end

    %% Control laws

    if converter_mode == 0
        D_bot = 0.5*ones(3, 1);
        D_top = zeros(3, 1);
    else
        D_bot = zeros(3, 1);
        D_top = 0.5*ones(3, 1);
    end
    u = [D_bot; D_top];

    %% Prediction step
    [x_pred, theta_pred, Px_pred, Ptheta_pred, Gamma_pred] = predict_state_theta( ...
        x_hat, u, theta_hat, Px_pred, Ptheta_pred, Gamma_pred, Gamma_hat, Qx, Kx, Ktheta, lambda_x, lambda_t, n_meas, n_theta, Ts, converter_mode);

    %% Debug variables
    debug_obs = zeros(40, 1);
    debug_obs(1:5) = y - x_pred;
    debug_obs(6:10) = y - x_hat;
    debug_obs(11:13) = IL;
    debug_obs(14) = Vout;
    debug_obs(15) = Vin;
    debug_obs(16) = G_load;
    debug_obs(17) = Idc_load;
    debug_obs(18) = CP_load;
    debug_obs(19:21) = L;
    debug_obs(22) = C;
    debug_obs(23:25) = RL;
    debug_obs(26:28) = VD;

    debug_deriv = zeros(25, 1);
    debug_deriv(1:3) = IL_dot;
    debug_deriv(4) = Vout_dot;

    debug_ctrl = zeros(25, 1);
    debug_ctrl(1:6) = u;
    debug_ctrl(7) = int_eVout;
end

function [x_dot] = model_interleaved_converter(x, u, converter_mode)
    IL = x(1:3);
    Vout = x(4);
    Vin = x(5);

    if converter_mode == 0
        d_bot = u(1:3);
        x_dot = [diag(Vin - (1 - d_bot)*Vout) diag(-IL) diag(-(1 - d_bot)) zeros(3, 4);
               zeros(1, 9) sum(IL.*(1 - d_bot)) -Vout -1 -1/(Vout + 0.001);
               zeros(1, 13)];
    else
        d_top = u(4:6);
        x_dot = [diag(Vin*d_top - Vout) diag(-IL) diag(-(1 - d_top)) zeros(3, 4);
               zeros(1, 9) sum(IL) -Vout -1 -1/(Vout + 0.001);
               zeros(1, 13)];
    end
end

%% Predict State and theta
function [x_pred, theta_pred, Px_pred, Ptheta_pred, Gamma_pred] = predict_state_theta( ...
    x, u, theta, Px, Ptheta, Gamma_pred, Gamma, Qx, Kx, Ktheta, lambda_x, lambda_t, n_meas, n_theta, Ts, converter_mode)

    [x_dot] = model_interleaved_converter(x, u, converter_mode);
    Psi = x_dot*Ts;

    x_pred = x + Psi*theta;

    theta_pred = theta;

    Px_pred = (1 / lambda_x) * (eye(n_meas) - Kx) * Px + Qx;

    Ptheta_pred = (1 / lambda_t) * (eye(n_theta) - Ktheta * Gamma_pred)*Ptheta;

    Gamma_pred = Gamma - Psi;
    
    % Numerical stabilization
    Px_pred = 0.5 * (Px_pred + Px_pred.');
    Ptheta_pred = 0.5 * (Ptheta_pred + Ptheta_pred.');
end

%% Update
function [x_update, theta_update, Gamma_update, Kx, Ktheta] = update_state_theta( ...
    y, x_pred, theta_pred, Px, Ptheta, Gamma_pred, R_y, R_theta, n_meas)

    % Innovation
    innovation = y - x_pred;

    % State correction gain
    Kx = Px / (Px + R_y);

    % Parameter correction gain
    Omega = Gamma_pred * Ptheta * Gamma_pred.' + R_theta;

    % Safer than explicit inverse
    Ktheta = Ptheta * Gamma_pred.' / Omega;
    % Ktheta = zeros(length(theta_pred), n_meas);

    % Corrected sensitivity
    Gamma_update = (eye(n_meas) - Kx) * Gamma_pred;

    % State correction
    x_update = x_pred + (Kx + Gamma_update * Ktheta) * innovation;

    % Parameter correction
    %
    % The negative sign follows the observer convention used in the paper,
    % where Gamma represents sensitivity of the state-estimation error.
    theta_update = theta_pred - Ktheta * innovation;
end