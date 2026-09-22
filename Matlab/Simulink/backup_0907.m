function [D_bot, D_top, debug_obs, debug_deriv, debug_ctrl] = fcn(Vin_m, Vout_m, IL_m, converter_mode, Vout_ref, Ts, control_mode, n_phases)
%#codegen
    persistent lambda alpha beta kappa Wm Wc ...
        x_pred P_pred sigma_pred ...
        Q R_meas ...
        integ_IL_est_alg_lin integ_Vout_est_alg_lin ...
        IL_dot_est_alg_lin Vout_dot_est_alg_lin ...
        RL VD ...
        G_load Idc_load CP_load ...
        n_est ...
        D_bot_last D_top_last ...
        int_eVout initialized

    %% Fixed dimensions
    n_states = 8;
    n_meas  = 5;
    n_sigma = 2*n_states + 1;
    n_est_alg = 100;

    %% System constants
    L   = 80e-3;
    C   = 12e-3;

    %% Initialization
    if isempty(initialized)
        %% UKF parameters
        alpha = 1e-3;
        beta  = 2.0;
        kappa = 0.0;

        %% Sigma weights
        lambda = (n_states + kappa)*alpha^2 - n_states;

        Wm = zeros(n_sigma, 1);
        Wc = zeros(n_sigma, 1);

        Wm(1) = lambda / (n_states + lambda);
        Wc(1) = lambda / (n_states + lambda) + (1 - alpha^2 + beta);

        for i = 2:n_sigma
            Wm(i) = 1 / (2 * (n_states + lambda));
            Wc(i) = 1 / (2 * (n_states + lambda));
        end

        %% Initial state estimate
        x0 = zeros(n_states, 1);

        x0(1) = IL_m(1);
        x0(2) = IL_m(2);
        x0(3) = IL_m(3);
        x0(4) = Vout_m;
        x0(5) = Vin_m;
        x0(6) = 0.0; % G_load
        x0(7) = 0.0; % Idc_load
        x0(8) = 0.0; % CP_load

        x_pred = x0;

        %%
        integ_IL_est_alg_lin = zeros(3, 1);
        IL_dot_est_alg_lin = zeros(3, 1);
        integ_Vout_est_alg_lin = 0;
        Vout_dot_est_alg_lin = 0;
        RL = 10e-3*ones(3, 1);
        VD  = 0.8*ones(3, 1);
        G_load = 0;
        Idc_load = 0;
        CP_load = 0;
        n_est = 0;

        %% Initial covariance
        P_pred = diag([
            1.0^2;     % iL1 uncertainty
            1.0^2;     % iL2 uncertainty
            1.0^2;     % iL3 uncertainty
            1.0^2;     % Vout uncertainty
            1.0^2;     % Vin uncertainty
            1.0^2;     % Gload uncertainty
            1.0^2;     % Idc_load uncertainty
            1.0^2      % CP_load uncertainty
        ]);

        %% Process noise covariance
        Q = diag([
            1e-3^2;     % iL1 model uncertainty
            1e-3^2;     % iL2 model uncertainty
            1e-3^2;     % iL3 model uncertainty
            1e-3^2;     % Vout model uncertainty
            1e-2^2;     % Vin random-walk/filtering noise
            1e-2^2;     % Gload random-walk noise
            1e-2^2;     % Idc_load random-walk noise
            1e-2^2      % CP_load random-walk noise
        ]);

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

        %% 
        D_bot_last = zeros(3, 1);
        D_top_last = zeros(3, 1);

        %% Generate initial sigma points around initial prediction
        sigma_pred = generate_sigma_points(x_pred, P_pred, n_states, n_sigma, alpha, kappa);

        %% Integral term
        int_eVout = 0.0;

        initialized = true;
    end

    %% Measurement vector at current sample
    z_meas = zeros(n_meas, 1);
    z_meas(1) = IL_m(1);
    z_meas(2) = IL_m(2);
    z_meas(3) = IL_m(3);
    z_meas(4) = Vout_m;
    z_meas(5) = Vin_m;

    %% UKF update step
    [x_hat, P_hat] = ukf_update_partial_state( ...
        x_pred, P_pred, sigma_pred, n_states, n_meas, n_sigma, Wm, Wc, z_meas, R_meas);

    %% Apply physical constraints
    x_hat(1) = max(x_hat(1), 0.0);
    x_hat(2) = max(x_hat(2), 0.0);
    x_hat(3) = max(x_hat(3), 0.0);
    x_hat(4) = max(x_hat(4), 0.0);
    x_hat(5) = max(x_hat(5), 0.0);
    x_hat(6) = max(x_hat(6), 0.0);
    x_hat(7) = max(x_hat(7), 0.0);
    x_hat(8) = max(x_hat(8), 0.0);

    %% Extract updated states
    IL       = x_hat(1:3);
    Vout     = x_hat(4);
    Vin      = x_hat(5);
    G_load_ukf   = x_hat(6);
    Idc_load_ukf = x_hat(7);
    CP_load_ukf  = x_hat(8);

    %% Low pass filter for load estimatives
    n_f = 9;
    alpha_f = 1/(1 + n_f);

    G_load = G_load + alpha_f*(G_load_ukf - G_load);
    Idc_load = Idc_load + alpha_f*(Idc_load_ukf - Idc_load);
    CP_load = CP_load + alpha_f*(CP_load_ukf - CP_load);

    %% Load Current
    I_load = G_load*Vout + Idc_load + CP_load/(Vout + 0.001);
    I_load_ref = G_load*Vout_ref + Idc_load + CP_load/Vout_ref;

    %% Derivatives
    % By model
    IL_dot_model = zeros(3, 1);
    if converter_mode == 0
        ID_sum = 0.0;
        for i = 1:n_phases
            ID = (1.0 - D_bot_last(i))*IL(i);
            IL_dot_model(i) = (Vin - RL(i)*IL(i) - (1.0 - D_bot_last(i))*(Vout + VD(i)))/L;
            ID_sum = ID_sum + ID;
        end
        Vout_dot_model = (ID_sum - I_load)/C;
    else
        for i = 1:n_phases
            IL_dot_model(i) = -(VD(i) + Vout + RL(i)*IL(i) - (Vin + VD(i))*D_top_last(i))/L;
        end
        Vout_dot_model = (sum(IL) - I_load)/C;
    end

    IL_dot = IL_dot_model;
    Vout_dot = Vout_dot_model;

    %% Power equations
    Pin = Vin*(IL(1) + IL(2) + IL(3));
    Pout = Vout*I_load;
    PR = RL(1)*IL(1)^2 + RL(2)*IL(2)^2 + RL(3)*IL(3)^2;
    PD = VD(1)*IL(1)*(1 - D_bot_last(1)) + VD(2)*IL(2)*(1 - D_bot_last(2)) + VD(3)*IL(3)*(1 - D_bot_last(3));
    PL = L*IL(1)*IL_dot(1) + L*IL(2)*IL_dot(2) + L*IL(3)*IL_dot(3);
    PC = C*Vout*Vout_dot;

    %% Algebraic estimator for IL to better estimate RL and VD

    t_integ = n_est_alg*Ts;
    tau = n_est*Ts;
    w = 2;
    if n_est == n_est_alg
        w = 1;
    end
    integ_IL_est_alg_lin = integ_IL_est_alg_lin + w*(t_integ - 2*tau)*IL_m;
    integ_Vout_est_alg_lin = integ_Vout_est_alg_lin + w*(t_integ - 2*tau)*Vout_m;

    if n_est == n_est_alg
        n_est = 0;
        tau = 0;

        integ_IL_est_alg_lin = integ_IL_est_alg_lin*Ts/2;
        IL_dot_est_alg_lin = -6*integ_IL_est_alg_lin/(t_integ^3);
        integ_IL_est_alg_lin = w*(t_integ - 2*tau)*IL_m;

        integ_Vout_est_alg_lin = integ_Vout_est_alg_lin*Ts/2;
        Vout_dot_est_alg_lin = -6*integ_Vout_est_alg_lin/(t_integ^3);
        integ_Vout_est_alg_lin = w*(t_integ - 2*tau)*Vout_m;

        %% MRAC to better estimate RL and VD
        if converter_mode == 0
            y_ILdot = Vin - IL_dot_est_alg_lin*L - Vout*(1 - D_bot_last);            
            phi_IL_dot = [diag(IL) diag((1 - D_bot_last));
                          IL'.^2 IL'.*(1 - D_bot_last')];
        else
            y_ILdot = Vin*D_top_last - Vout - IL_dot_est_alg_lin*L;
            phi_IL_dot = [diag(IL) diag((1 - D_top_last));
                          IL'.^2 IL'.*(1 - D_bot_last')];
        end

        PL_est = L*(IL'*IL_dot_est_alg_lin);
        PC_est = C*Vout*Vout_dot_est_alg_lin;
        y_P = Pin - Pout - PL_est - PC_est;

        y = [y_ILdot; y_P];
        
        theta_mdl = [RL; VD];
        
        y_hat = phi_IL_dot*theta_mdl;
        err_mdl = y - y_hat;
        err_mdl(end) = err_mdl(end)/1000;

        gamma_RL = 0.01*ones(3, 1);
        gamma_VD = 0.01*ones(3, 1);
        gamma = diag([gamma_RL; gamma_VD]);
        theta_mdl_dot = gamma*(phi_IL_dot')*err_mdl;

        theta_mdl = theta_mdl + n_est_alg*Ts*theta_mdl_dot;
        RL = min(max(theta_mdl(1:3), 1e-4), 1e-1);
        VD = min(max(theta_mdl(4:6), 0.7), 1.5);
    end
    n_est = n_est + 1;
    
    %% Voltage error and integral
    e_Vout = Vout_ref - Vout;

    if abs(e_Vout) < 0.5*max(Vout_ref, 1.0)
        int_eVout = int_eVout + e_Vout * Ts;
    end

    %% Evaluate expected load current at Vref and the equilibrium inductor current
    % Equilibrium current
    I_load_Vref = G_load*Vout_ref + Idc_load + CP_load/Vout_ref;
    I_load_Vref = min(max(0, I_load_Vref), 200);
    I_load = G_load*Vout + Idc_load + CP_load/Vout;

    R_L = mean(RL);
    V_D = mean(VD);
    if converter_mode == 0
        discriminant = (n_phases*Vin)^2 - 4*n_phases*R_L*I_load_Vref*(V_D + Vout);
        if discriminant > 0
            IL_eq = Vin/(2*R_L) - sqrt((n_phases*Vin)^2 - 4*n_phases*R_L*I_load_Vref*(V_D + Vout))/(2*n_phases*R_L);
        else
            IL_eq = Vin/(2*R_L);
        end
    elseif converter_mode == 1
        IL_eq = I_load_Vref/n_phases;
    else
        IL_eq = 0;
    end
    IL_ref = IL_eq;

    %% Control laws
    % dictionary("IDAPBC", 0, "Krasovskii", 1, "CascadeModelBased", 2, "CascadeModelFree", 3);
    k = 0.1;
    ts = 0.2;
    zeta = 0.707;

    gains = zeros(3, 1);

    if converter_mode == 0
        D_top = zeros(3, 1);

        if control_mode == 0 % IDAPBC
            gains = IDAPBC_gains_Boost(C,L,R_L,k,ts,zeta); % gains = [Kj_conv; Kr_conv; Ki_conv];
            Kj = gains(1);
            Kr = gains(2);
            Ki = gains(3);
            [d_num,d_den] = IDAPBC_duty_Boost(C,IL(1),IL(2),IL(3),I_load_Vref,Ki,Kj,Kr,L,R_L,V_D,Vin,Vout,int_eVout);

            D_bot = d_num(:)./d_den;
        elseif control_mode == 1 % Krasovskii
            d_ref = (V_D - Vin + Vout_ref + R_L*IL_eq)/(V_D + Vout_ref);

            gains = Krasovskii_gains_Boost(C,IL_eq,L,R_L,Vout_ref,d_ref,k,ts,zeta);
            % gains = [K_u_conv; Kp_v_conv; Kp_i_conv];

            K_u = gains(1);
            Kp_v = gains(2);
            Kp_i = gains(3);

            d_dot = zeros(3, 1);
            for i = 1:n_phases
                d_dot(i) = Krasovskii_duty_Boost(IL(i),IL_eq,IL_dot(i),K_u,Kp_i,Kp_v,V_D,Vout,Vout_ref,Vout_dot);
            end

            D_bot = D_bot_last(:, end) + d_dot*Ts;
        elseif control_mode == 2 || control_mode == 3
            gains = Cascade_gains_Boost(C,IL_eq,L,V_D,Vin,Vout_ref,k,ts,zeta);
            % gains = [Kp_i_conv; Kp_v_conv; Ki_v_conv];

            Kp_i = gains(1);
            Kp_v = gains(2);
            Ki_v = gains(3);

            IL_ref = IL_eq + Kp_v*(e_Vout + Ki_v*int_eVout);

            D_bot = zeros(3, 1);
            if control_mode == 2 % Cascade model based
                for i = 1:n_phases
                    D_bot(i) = 1 - (Vin - IL(i).*R_L + Kp_i.*L.*(IL(i) - IL_ref))./(V_D+Vout);
                    % D_bot(i) = Cascade_duty_Boost(IL(i),IL_eq,Ki_v,Kp_i,Kp_v,L,RL,VD,Vin,Vout,Vout_ref,int_eVout);
                end
            else % Cascade model free
                
            end
        else
            D_bot = 0.5*ones(3, 1);
            for i = 1:n_phases
                D_bot(i) = 1 - (Vin - IL_eq.*RL(i))./(VD(i) + Vout_ref);
            end
        end

        %% Saturate duties
        D_bot = min(max(D_bot, 0.0), 1.0);
        D_bot_last = D_bot;
    else
        D_bot = zeros(3, 1);
        D_top = zeros(3, 1);

        if control_mode == 0 % IDAPBC

        elseif control_mode == 1 % Krasovskii

        elseif control_mode == 2 || control_mode == 3

        else
            D_top = 0.5*ones(3, 1);
        end

        %% Saturate duties
        D_top = min(max(D_top, 0.0), 1.0);
        D_top_last = D_top;
    end

    %% UKF predicition step
    [x_pred, P_pred, sigma_pred] = ukf_predict( ...
            x_hat, P_hat, n_states, n_sigma, Q, D_top, n_phases, Ts, alpha, kappa, Wm, Wc, L, C, RL, VD, converter_mode);

    %% Debug variables
    debug_obs = zeros(25, 1);
    debug_obs(1:3) = IL;
    debug_obs(4) = Vout;
    debug_obs(5) = Vin;
    debug_obs(6) = I_load;
    debug_obs(7) = G_load;
    debug_obs(8) = Idc_load;
    debug_obs(9) = CP_load;
    debug_obs(10) = I_load_ref;
    debug_obs(11:13) = RL;
    debug_obs(14:16) = VD;
    debug_obs(17) = Pin;
    debug_obs(18) = Pout;
    debug_obs(19) = PR;
    debug_obs(20) = PD;
    debug_obs(21) = PL;
    debug_obs(22) = PC;

    debug_deriv = zeros(11, 1);
    debug_deriv(1:3) = IL_dot_est_alg_lin;
    debug_deriv(4:6) = IL_dot_model;
    debug_deriv(7) = Vout_dot_model;
    debug_deriv(8) = Vout_dot_est_alg_lin;

    debug_ctrl = zeros(10, 1);
    debug_ctrl(1) = IL_eq;
    debug_ctrl(2) = Vout_ref;
    debug_ctrl(3) = IL_ref;
    debug_ctrl(4) = int_eVout;
    debug_ctrl(5:7) = gains(:);
end

function x_next = discrete_model(x, d, n_phases, n_states, converter_mode, Ts, L, C, RL, VD)
    IL     = x(1:3);
    Vout   = x(4);
    Vin    = x(5);
    G_load = x(6);
    Idc_load = x(7);
    CP_load = x(8);

    I_load = G_load*Vout + Idc_load + CP_load/(Vout + 0.001);

    diL = zeros(3, 1);
    if converter_mode == 0
        ID_sum = 0.0;
        for i = 1:3
            if i <= n_phases
                ID = (1.0 - d(i))*IL(i);
                diL(i) = (Vin - RL(i)*IL(i) - (1.0 - d(i))*(Vout + VD(i))) / L;
                ID_sum = ID_sum + ID;
            else
                %% Force inactive current toward zero in one sample
                diL(i) = -IL(i)/Ts;
            end
        end
        dVout = (ID_sum - I_load)/C;
    else
        for i = 1:3
            if i <= n_phases
                diL(i) = -(VD(i) + Vout + RL(i)*IL(i) - (Vin + VD(i))*d(i))/L;
            else
                %% Force inactive current toward zero in one sample
                diL(i) = -IL(i)/Ts;
            end
        end
        dVout = (sum(IL) - I_load)/C;
    end

    %% Random walk states
    dVin = 0.0;
    dGload = 0.0;
    dIdc_load = 0.0;
    dCP_load = 0.0;

    x_next = zeros(n_states, 1);

    x_next(1) = IL(1) + Ts*diL(1);
    x_next(2) = IL(2) + Ts*diL(2);
    x_next(3) = IL(3) + Ts*diL(3);
    x_next(4) = Vout  + Ts*dVout;
    x_next(5) = Vin   + Ts*dVin;
    x_next(6) = G_load + Ts*dGload;
    x_next(7) = Idc_load + Ts*dIdc_load;
    x_next(8) = CP_load + Ts*dCP_load;
end

function [x_pred, P_pred, sigma_pred] = ukf_predict( ...
    x, P, n_states, n_sigma, Q, d, n_phases, Ts, alpha, kappa, Wm, Wc, L, C, RL, VD, converter_mode)

    %% Generate sigma points
    sigma = generate_sigma_points(x, P, n_states, n_sigma, alpha, kappa);

    %% Propagate sigma points
    sigma_pred = zeros(n_states, n_sigma);

    for i = 1:n_sigma
        sigma_pred(:, i) = discrete_model( ...
                sigma(:, i), d, n_phases, n_states, converter_mode, Ts, L, C, RL, VD);
    end

    %% Predicted mean
    x_pred = zeros(n_states, 1);

    for i = 1:n_sigma
        x_pred = x_pred + Wm(i) * sigma_pred(:, i);
    end

    %% Predicted covariance
    P_pred = zeros(n_states, n_states);

    for i = 1:n_sigma
        dx = sigma_pred(:, i) - x_pred;
        P_pred = P_pred + Wc(i) * (dx * dx');
    end

    P_pred = P_pred + Q;

    %% Symmetrize
    P_pred = 0.5*(P_pred + P_pred');
end

function [x_update, P_update] = ukf_update_partial_state( ...
    x_pred, P_pred, sigma_pred, n_states, n_meas, n_sigma, Wm, Wc, z, R_meas)

    %% Measurement sigma points
    Z_sigma = zeros(n_meas, n_sigma);

    for i = 1:n_sigma
        Z_sigma(:, i) = measurement_model(sigma_pred(:, i), n_meas);
    end

    %% Predicted measurement mean
    z_pred = zeros(n_meas, 1);

    for i = 1:n_sigma
        z_pred = z_pred + Wm(i) * Z_sigma(:, i);
    end

    %% Innovation covariance
    S = zeros(n_meas, n_meas);

    for i = 1:n_sigma
        dz = Z_sigma(:, i) - z_pred;
        S = S + Wc(i) * (dz * dz');
    end

    S = S + R_meas;

    %% Numerical stabilization
    S = 0.5 * (S + S') + 1e-9 * eye(n_meas);

    %% Cross covariance
    Pxz = zeros(n_states, n_meas);

    for i = 1:n_sigma
        dx = sigma_pred(:, i) - x_pred;
        dz = Z_sigma(:, i) - z_pred;
        Pxz = Pxz + Wc(i) * (dx * dz');
    end

    %% Kalman gain
    K = Pxz/S;

    %% Innovation
    y = z - z_pred;

    %% Update
    x_update = x_pred + K*y;
    P_update = P_pred - K*S*K';

    %% Symmetrize
    P_update = 0.5 * (P_update + P_update');
end

function z = measurement_model(x, n_meas)
    %% Measurements:
    %% z = [iL1; iL2; iL3; Vout; Vin]
    %% Iload is not directly measured.
    z = x(1:n_meas);
end

function sigma = generate_sigma_points(x, P, n_states, n_sigma, alpha, kappa)
    lambda = alpha^2 * (n_states + kappa) - n_states;

    sigma = zeros(n_states, n_sigma);
    sigma(:, 1) = x;

    %% Numerical stabilization
    P = 0.5 * (P + P');

    jitter = 1e-9;
    success = false;
    S = zeros(n_states, n_states);

    for attempt = 1:8
        [S_temp, flag] = chol((n_states + lambda)*P + jitter*eye(n_states), 'lower');

        if flag == 0
            S = S_temp;
            success = true;
            break;
        else
            jitter = jitter * 10.0;
        end
    end

    if ~success
        %% Last fallback for simulation robustness
        P = P + 1e-6*eye(n_states);
        S = chol((n_states + lambda) * P, 'lower');
    end

    for i = 1:n_states
        sigma(:, i + 1)     = x + S(:, i);
        sigma(:, i + 1 + n_states) = x - S(:, i);
    end
end

function [conv_U] = algEst_conv_U(Ts, U_last)
    N = length(U_last) - 1;
    taus_int = Ts.*(0:N)';
    w_int = [1 2*ones(1, N - 1) 1];
    t_int = N*Ts;

    f_int = (t_int - taus_int).*taus_int.*U_last;
    int_a1 = w_int*f_int*Ts/2;
    conv_U = -6*int_a1/(t_int^3);
end