function [D_bot, D_top, SW_LV, SW_HV, I_load, I_load_Vref, IL_eq, IL_ref, Kp_v, Ti_v, integ_eVout, u_dot, d_ref, theta_mdl, theta_load_hat] = fcn(Vin_m, Vout_m, IL_m, converter_mode, n_phases, Vout_ref, Ts, control_mode, settling_time_i, N_est_alg_Vout, N_est_alg_load, N_est_alg_IL, N_est_alg_U, zeta_v, ratio_ts, N_est_alg_Vin)
    persistent D_bot_last D_top_last Vin_last Vout_last IL_last IL_load_last int_eVout theta_load invR_load L C R_L VD
    if isempty(D_bot_last)
        D_bot_last = zeros(3, N_est_alg_U + 1);
        D_top_last = zeros(3, N_est_alg_U + 1);
        Vin_last = zeros(N_est_alg_Vin + 1, 1);
        Vout_last = zeros(N_est_alg_Vout + 1, 1);
        IL_last = zeros(N_est_alg_IL + 1, n_phases);
        IL_load_last = zeros(N_est_alg_load + 1, 1);
        int_eVout = 0;
        theta_load = zeros(3, 1);
        invR_load = 1*eye(3);
        R_L = 10e-3;
        VD = 0.8;
        L = 80e-3;
        C = 12e-3;
    end

    %% Parameters
    P_min = 100;
    Idc_load_min = P_min/Vout_ref;
    G_load_min = P_min/(Vout_ref^2);
    CP_load_min = P_min;
    param_min = [Idc_load_min; G_load_min; CP_load_min];

    P_max = 15000; 
    Idc_load_max = P_max/Vout_ref;
    G_load_max = P_max/(Vout_ref^2);
    CP_load_max = P_max;
    param_max = [Idc_load_max; G_load_max; CP_load_max];

    %% Estimate Vin and its time derivative
    Vin_last = [Vin_last(2:end); Vin_m];
    [Vin, ~] = algEst_lin(Ts, Vin_last);

    %% Estimate Vout and its time derivative
    Vout_last = [Vout_last(2:end); Vout_m];
    [Vout, Vout_dot] = algEst_lin(Ts, Vout_last);

    %% Excitacao persistente, Erros e integral do erro de tensao
    e_Vout = Vout - Vout_ref;
    if abs(e_Vout) < 0.5*Vout_ref && Vout_last(end-1) ~= 0
        e_Vout_last = Vout_last(end-1) - Vout_ref;
        int_eVout = int_eVout + (e_Vout + e_Vout_last)*Ts/2;
    end

    %% Estimate IL and its time derivative
    IL_last = [IL_last(2:end, :); IL_m'];

    IL = zeros(3, 1);
    IL_dot = zeros(3, 1);
    for i = 1:n_phases
        [IL(i), IL_dot(i)] = algEst_lin(Ts, IL_last(:, i));
    end

    %% MRAC to better estimate RL and VD
    if converter_mode == 0
        % IL_dot = (Vin - (VD + Vout)*(1 - d) - R_L*IL)/L
        % IL_dot*L = Vin - Vout*(1 - d) - R_L*IL - VD*(1 - d)
        % Vin - IL_dot*L - Vout*(1 - d) = R_L*IL + VD*(1 - d)
        % y = R_L*IL + (Vd0 + Rd*IL*(1 - d))*(1 - d)
        % y = R_L*IL + Vd0*(1 - d) + Rd*IL*(1 - d)²
        % phi_IL_dot = [IL, 1 - D_bot_last(:, end), IL.*(1 - D_bot_last(:, end)).^2];
        % phi_IL_dot = [IL, 1 - D_bot_last(:, end), IL.*(1 - D_bot_last(:, end)).^2];
        phi_IL_dot = [IL, 1 - D_bot_last(:, end)];

        % y = Vin - IL_dot*L - Vout*(1 - D_bot_last(:, end));
        y = Vin - IL_dot*L - Vout*(1 - D_bot_last(:, end));
    elseif converter_mode == 1
        % IL_dot = ((VD + Vin)*d - VD - Vout - R_L*IL)/L
        % IL_dot*L = (VD + Vin)*d - VD - Vout - R_L*IL
        % Vin*d - Vout - IL_dot*L = R_L*IL + VD*(1 - d)
        % y = R_L*IL + (Vd0 + Rd*IL*(1 - d))*(1 - d)
        % y = R_L*IL + Vd0*(1 - d) + Rd*IL*(1 - d)²
        % phi_IL_dot = [IL, 1 - D_top_last(:, end), IL.*(1 - D_top_last(:, end)).^2];
        % phi_IL_dot = [IL, 1 - D_top_last(:, end), IL.*(1 - D_top_last(:, end)).^2];
        phi_IL_dot = [IL, 1 - D_top_last(:, end)];

        % y = Vin*D_top_last(:, end) - Vout - IL_dot*L;
        y = Vin*D_top_last(:, end) - Vout - IL_dot*L;
    else
        phi_IL_dot = zeros(1, 2);
        y = zeros(3, 1);
    end

    theta_mdl = [R_L; VD];
    y_hat = phi_IL_dot*theta_mdl;
    
    err_mdl = y - y_hat;

    gamma = diag([0.01 0.1]);
    theta_mdl_dot = gamma*(phi_IL_dot')*err_mdl;

    theta_mdl = theta_mdl + Ts*theta_mdl_dot;

    % R_L = min(max(theta_mdl(1), 1e-5), 1);
    % VD = min(max(theta_mdl(2), 0.3), 1.5);

    %% Estimate load current
    if converter_mode == 0
        IL_load_last = [IL_load_last(2:end); 
                        sum(IL_m.*(1 - D_bot_last(:, end)))];
        [IL_load, ~] = algEst_lin(Ts, IL_load_last);

        I_load = IL_load - C*Vout_dot;
    elseif converter_mode == 1
        I_load = sum(IL) - C*Vout_dot;
    else
        I_load = 0;
    end

    %% Generate relay signals
    SW_LV = 1; %1*(Vin > 5);
    SW_HV = 1; %1*(Vout > 0.95*Vref);

    %% RLS with forgetting factor
    if Vout_dot^2 < 100
        lambda_load = 1;
    else
        lambda_load = 0.99; % Forgetting factor
    end
    eps_Vout = 0.01; % Small value to avoid zero division
    phi = [1; Vout; 1/(Vout + eps_Vout)]; % I_load = Idc + G*Vout + CP/Vout

    [theta_load, invR_load] = RLS_iter(theta_load, invR_load, phi, I_load, lambda_load);

    theta_load = theta_load.*(theta_load < param_max) + param_max.*(theta_load >= param_max);
    theta_load_hat = theta_load.*(theta_load > param_min);
    
    %% Evaluate expected load current at Vref and the equilibrium inductor current

    % Equilibrium current
    I_load_Vref = [1 Vout_ref 1/Vout_ref]*theta_load_hat;
    I_load_Vref = min(max(0, I_load_Vref), 200);

    if converter_mode == 0
        % -(3*R_L*IL_eq^2 - 3*Vin*IL_eq + I_load*VD + I_load*Vout)/(C*(VD + Vout))
        % -(n*R_L*IL_eq^2 - n*Vin*IL_eq + I_load*(VD + Vout))/(C*(VD + Vout))

        % a = n*R_L; b = -n*Vin; c = I_load*(VD + Vout)
        % for delta to be positive: b² - 4ac > 0
        % n²Vin² - 4*n*R_L*I_load*(VD + Vout) > 0
        % n²*Vin² > 4*n*R_L*I_load*(VD + Vout)

        % General root will be:
        % -b/2a 
        
        if ((n_phases*Vin)^2 > 4*n_phases*R_L*I_load_Vref*(VD + Vout))
            IL_eq = Vin/(2*R_L) - sqrt((n_phases*Vin)^2 - 4*n_phases*R_L*I_load_Vref*(VD + Vout))/(2*n_phases*R_L);
        else
            IL_eq = Vin/(2*R_L) - sqrt((n_phases*Vin)^2 - 4*n_phases*R_L*I_load*(VD + Vout))/(2*n_phases*R_L);
        end
    elseif converter_mode == 1
        IL_eq = I_load_Vref/n_phases;
    else
        IL_eq = 0;
    end

    % IL_eq = IL_ref_expr(I_load_Vref,R_L,VD,Vin,Vout,converter_mode,n_phases);

    %% Control Laws
    D_bot = zeros(3, 1);
    D_top = zeros(3, 1);

    Kp_v = 0;
    Ti_v = 0;

    d_ref = 0;
    u_dot = 0;
    if control_mode == 0 || control_mode == 1
        % Evaluate current inductor reference and its error
        IL_ref = min(max(IL_eq, 0), 100);
        e_IL = IL - IL_ref;

        if control_mode == 0 % IDAPBC
        elseif control_mode == 1 % Krasovskii
            tau_d = 0.05/4; %1*settling_time_i/4;
            Kp_d = 1/tau_d;
            K_p = 0.001;
            if converter_mode == 0
                d_ref = 1 - (Vin - R_L*IL_ref)/(VD + Vout_ref);
                for i = 1:n_phases
                    e_u = D_bot_last(i, end) - d_ref;
                    e_u_last = D_bot_last(i, end-1) - d_ref;
                    u_dot = K_p.*(IL(i).*Vout_dot - IL_dot(i).*(Vout + VD)) ...
                        - 0.12*e_Vout - Kp_d*(e_u + e_u_last)/2;
                    
                    D_bot(i) = D_bot_last(i, end) + u_dot*Ts;
                end
            elseif converter_mode == 1
                d_ref = (VD + Vout_ref + R_L*IL_ref)/(VD + Vin);
                for i = 1:n_phases
                    e_u = D_top_last(i, end) - d_ref;
                    e_u_last = D_top_last(i, end-1) - d_ref;
                    u_dot = -Kp_d*(e_u + e_u_last)/2 + ...
                        K_p.*(-IL_dot(i).*(Vin + VD));
                    D_top(i) = D_top_last(i, end) + u_dot*Ts;
                end
            end
        end
    elseif control_mode == 2 || control_mode == 3
        %% Cascade
        tau_i = settling_time_i/4;
        Kp_i = 1/tau_i;
    
        settling_time_v = settling_time_i*ratio_ts;
        wn_v = 4/(zeta_v*settling_time_v);

        gains_cascade = gains_cascade_expr(C,IL_eq,Kp_i,L,VD,Vin,Vout_ref,converter_mode,wn_v,zeta_v);
        Kp_v = gains_cascade(1);
        Ti_v = gains_cascade(2);  

        % Evaluate current inductor reference and its error
        u_IL = Kp_v*(e_Vout + int_eVout/Ti_v);
        IL_ref = IL_eq - u_IL; 
        IL_ref = min(max(IL_ref, 0), 100);
    
        e_IL = IL - IL_ref;
        if control_mode == 2 % Cascade model based
            if converter_mode == 0
                for i = 1:n_phases
                    D_bot(i) = (VD + Vout - Vin + L*IL_dot(i) + R_L*IL(i) - L*Kp_i*e_IL(i))/(VD + Vout);
                end
            elseif (converter_mode == 1.0)
                for i = 1:n_phases
                    D_top(i) = (VD + Vout + L*IL_dot(i) + R_L*IL(i) - Kp_i*L*e_IL(i))/(VD+Vin);
                end
            end
        elseif control_mode == 3 % Cascade model free
            if converter_mode == 0
                for i = 1:n_phases
                    conv_U = algEst_conv_U(Ts, D_bot_last(i, :)');
    
                    alpha = (VD + Vout)/L;
                    phi = IL_dot(i) + alpha*conv_U;
            
                    D_bot(i) = (-phi - Kp_i*e_IL(i))/alpha;
                end
            elseif converter_mode == 1
                for i = 1:n_phases
                    conv_U = algEst_conv_U(Ts, D_top_last(i, :)');
    
                    alpha = (VD + Vin)/L;
                    phi = IL_dot(i) + alpha*conv_U;
            
                    D_top(i) = (-phi - Kp_i*e_IL(i))/alpha;
                end
            end
        end
    else
        IL_ref = 0;
    end

    D_bot_last = [D_bot_last(:, 2:end) min(max(D_bot, 0), 1)];
    D_top_last = [D_top_last(:, 2:end) min(max(D_top, 0), 1)];
    integ_eVout = int_eVout;
end

function [X, X_dot] = algEst_lin(Ts, X_last)
    N = length(X_last) - 1;
    taus_int = Ts.*(0:N)';
    w_int = [1 2*ones(1, N - 1) 1];
    t_int = N*Ts;

    f_int = (2*t_int - 3.*taus_int).*X_last;
    int_a0 = w_int*f_int*Ts/2;
    a0 = 2*int_a0/(t_int^2);

    f_int = (t_int - 2.*taus_int).*X_last;
    int_a1 = w_int*f_int*Ts/2;
    a1 = -6*int_a1/(t_int^3);

    X = a0 + a1*t_int;
    X_dot = a1;
end

function [X, X_dot, X_ddot] = algEst_quad(Ts, X_last) %#ok<DEFNU>
    N = length(X_last) - 1;
    taus_int = Ts.*(0:N)';
    w_int = [1 2*ones(1, N - 1) 1];
    t_int = N*Ts;

    f_int = (3*t_int.^2 - 12*t_int.*taus_int + 10.*taus_int.^2).*X_last;
    int_a0 = w_int*f_int*Ts/2;
    a0 = 3*int_a0/(t_int^3);

    f_int = (3*t_int.^2 - 16*t_int.*taus_int + 15.*taus_int.^2).*X_last;
    int_a1 = w_int*f_int*Ts/2;
    a1 = -12*int_a1/(t_int^4);

    f_int = (t_int.^2 - 6*t_int.*taus_int + 6.*taus_int.^2).*X_last;
    int_a2 = w_int*f_int*Ts/2;
    a2 = 60*int_a2/(t_int^5);

    X = a0 + a1*t_int + 0.5*a2*t_int^2;
    X_dot = a1 + a2*t_int;
    X_ddot = a2;
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

function [theta_i, invR_i] = RLS_iter(theta_i_minus_1, invR_i_minus_1, phi, y, lambda)
    y_hat = (phi')*theta_i_minus_1;
    err = y - y_hat;

    Kk = invR_i_minus_1*phi/(lambda + (phi')*invR_i_minus_1*phi);
    invR_i = (1/lambda)*(eye - Kk*(phi'))*invR_i_minus_1;
    invR_i = (invR_i + invR_i')/2;

    theta_i = theta_i_minus_1 + Kk*err;
end