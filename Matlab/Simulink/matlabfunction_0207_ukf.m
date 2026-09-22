function [D_bot, D_top, I_load, IL, Vout, Vin] = fcn(Vin_m, Vout_m, IL_m, converter_mode, n_phases, Vout_ref, Ts)
%#codegen
    persistent alpha beta kappa ...
        Wm_sigma Wc_sigma ...
        x_pred P_pred sigma_pred ...
        Q R_meas ...
        int_eVout initialized

    %% Fixed dimensions
    n_states = 6;
    n_meas  = 5;
    n_sigma = 2*n_states + 1;

    %% System constants
    R_L = 10e-2;
    VD  = 0.7;
    L   = 80e-3;
    C   = 12e-3;

    %% Clamp number of active phases
    if n_phases < 1
        n_ph = 1;
    elseif n_phases > 3
        n_ph = 3;
    else
        n_ph = round(n_phases);
    end

    %% Initialization
    if isempty(initialized)

        %% UKF parameters
        alpha = 1e-3;
        beta  = 2.0;
        kappa = 0.0;

        %% Sigma weights
        lambda = alpha^2 * (n_states + kappa) - n_states;

        Wm_sigma = zeros(n_sigma, 1);
        Wc_sigma = zeros(n_sigma, 1);

        Wm_sigma(1) = lambda / (n_states + lambda);
        Wc_sigma(1) = lambda / (n_states + lambda) + (1 - alpha^2 + beta);

        for i = 2:n_sigma
            Wm_sigma(i) = 1 / (2 * (n_states + lambda));
            Wc_sigma(i) = 1 / (2 * (n_states + lambda));
        end

        %% Initial state estimate
        x0 = zeros(n_states, 1);

        x0(1) = IL_m(1);
        x0(2) = IL_m(2);
        x0(3) = IL_m(3);
        x0(4) = Vout_m;
        x0(5) = Vin_m;

        %% Initial load current estimate
        %% If no better value is known, start from zero.
        x0(6) = 0.0;

        x_pred = x0;

        %% Initial covariance
        P_pred = diag([
            1.0^2;     % iL1 uncertainty
            1.0^2;     % iL2 uncertainty
            1.0^2;     % iL3 uncertainty
            1.0^2;     % Vout uncertainty
            1.0^2;     % Vin uncertainty
            1.0^2      % Iload uncertainty
        ]);

        %% Process noise covariance
        Q = diag([
            0.01^2;     % iL1 model uncertainty
            0.01^2;     % iL2 model uncertainty
            0.01^2;     % iL3 model uncertainty
            0.01^2;     % Vout model uncertainty
            0.01^2;     % Vin random-walk/filtering noise
            0.01^2      % Iload random-walk noise
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

    %% Ignore inactive phase current measurements by increasing measurement noise
    R_now = R_meas;

    if n_ph < 3
        R_now(3,3) = 1e6;
    end

    if n_ph < 2
        R_now(2,2) = 1e6;
    end

    %% UKF update step
    [x_hat, P_hat] = ukf_update_partial_state( ...
        x_pred, P_pred, n_states, n_meas, n_sigma, sigma_pred, Wm_sigma, Wc_sigma, z_meas, R_now);

    %% Force inactive phases to zero after update
    if n_ph < 3
        x_hat(3) = 0.0;
        P_hat(3,:) = 0.0;
        P_hat(:,3) = 0.0;
        P_hat(3,3) = 1e-9;
    end

    if n_ph < 2
        x_hat(2) = 0.0;
        P_hat(2,:) = 0.0;
        P_hat(:,2) = 0.0;
        P_hat(2,2) = 1e-9;
    end

    %% Apply physical constraints
    x_hat(1) = max(x_hat(1), 0.0);
    x_hat(2) = max(x_hat(2), 0.0);
    x_hat(3) = max(x_hat(3), 0.0);
    x_hat(4) = max(x_hat(4), 0.0);
    x_hat(5) = max(x_hat(5), 0.0);
    x_hat(6) = max(x_hat(6), 0.0);

    %% Extract updated states
    IL     = x_hat(1:3);
    Vout   = x_hat(4);
    Vin    = x_hat(5);
    I_load = x_hat(6);

    %% Voltage error and integral
    e_Vout = Vout_ref - Vout;

    if abs(e_Vout) < 0.5*max(Vout_ref, 1.0)
        int_eVout = int_eVout + e_Vout * Ts;
    end

    %% Control laws
    if converter_mode == 0
        D_bot = 0.5 * ones(3, 1);
        D_top = zeros(3, 1);
    else
        D_bot = zeros(3, 1);
        D_top = 0.5 * ones(3, 1);
    end

    %% Disable inactive phases
    if n_ph < 3
        D_bot(3) = 0.0;
        D_top(3) = 0.0;
    end

    if n_ph < 2
        D_bot(2) = 0.0;
        D_top(2) = 0.0;
    end

    %% Saturate duties
    D_bot = min(max(D_bot, 0.0), 1.0);
    D_top = min(max(D_top, 0.0), 1.0);

    %% For this boost averaged model, bottom switch duty is used
    d = D_bot;

    %% UKF prediction step
    [x_pred_next, P_pred_next, sigma_pred_next, Wm_sigma, Wc_sigma] = ukf_predict( ...
        x_hat, P_hat, n_states, n_sigma, Q, d, n_ph, Ts, alpha, beta, kappa, L, C, R_L, VD);

    %% Force inactive predicted states to zero
    if n_ph < 3
        x_pred_next(3) = 0.0;
        P_pred_next(3,:) = 0.0;
        P_pred_next(:,3) = 0.0;
        P_pred_next(3,3) = 1e-9;
    end

    if n_ph < 2
        x_pred_next(2) = 0.0;
        P_pred_next(2,:) = 0.0;
        P_pred_next(:,2) = 0.0;
        P_pred_next(2,2) = 1e-9;
    end

    %% Store one-step-ahead prediction for next sample
    x_pred = x_pred_next;
    P_pred = P_pred_next;
    sigma_pred = sigma_pred_next;
end

function x_next = boost_discrete_model(x, d, n_phases, Ts, L, C, R_L, VD)
    IL     = x(1:3);
    Vout   = x(4);
    Vin    = x(5);
    I_load = x(6);

    diL = zeros(3, 1);
    diode_current_sum = 0.0;

    for j = 1:3
        if j <= n_phases
            diL(j) = (Vin - R_L*IL(j) - (1.0 - d(j))*(Vout + VD)) / L;
            diode_current_sum = diode_current_sum + (1.0 - d(j))*IL(j);
        else
            %% Force inactive current toward zero in one sample
            diL(j) = -IL(j) / Ts;
        end
    end

    dVout = (diode_current_sum - I_load) / C;

    %% Random walk states
    dVin = 0.0;
    dIload = 0.0;

    x_next = zeros(6, 1);

    x_next(1) = IL(1) + Ts*diL(1);
    x_next(2) = IL(2) + Ts*diL(2);
    x_next(3) = IL(3) + Ts*diL(3);
    x_next(4) = Vout  + Ts*dVout;
    x_next(5) = Vin   + Ts*dVin;
    x_next(6) = I_load + Ts*dIload;

end

function [x_pred, P_pred, sigma_pred, Wm, Wc] = ukf_predict( ...
    x, P, n_states, n_sigma, Q, d, n_phases, Ts, alpha, beta, kappa, L, C, R_L, VD)

    lambda = alpha^2 * (n_states + kappa) - n_states;

    %% Compute weights
    Wm = zeros(n_sigma, 1);
    Wc = zeros(n_sigma, 1);

    Wm(1) = lambda / (n_states + lambda);
    Wc(1) = lambda / (n_states + lambda) + (1 - alpha^2 + beta);

    for i = 2:n_sigma
        Wm(i) = 1 / (2 * (n_states + lambda));
        Wc(i) = 1 / (2 * (n_states + lambda));
    end

    %% Generate sigma points
    sigma = generate_sigma_points(x, P, n_states, n_sigma, alpha, kappa);

    %% Propagate sigma points
    sigma_pred = zeros(n_states, n_sigma);

    for i = 1:n_sigma
        sigma_pred(:, i) = boost_discrete_model( ...
            sigma(:, i), d, n_phases, Ts, L, C, R_L, VD);
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
    P_pred = 0.5 * (P_pred + P_pred');

end

function [x_update, P_update] = ukf_update_partial_state( ...
    x_pred, P_pred, n_states, n_meas, n_sigma, sigma_pred, Wm, Wc, z, R_meas)

    %% Measurement sigma points
    Z_sigma = zeros(n_meas, n_sigma);

    for i = 1:n_sigma
        Z_sigma(:, i) = measurement_model(sigma_pred(:, i));
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
    x_update = x_pred + K * y;

    P_update = P_pred - K*S*K';

    %% Symmetrize
    P_update = 0.5 * (P_update + P_update');

end

function z = measurement_model(x)
    %% Measurements:
    %% z = [iL1; iL2; iL3; Vout; Vin]
    %% Iload is not directly measured.
    z = x(1:5);
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