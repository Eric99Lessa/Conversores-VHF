clear; clc; close all;

%% ============================================================
%  BOOST CONVERTER UKF EXAMPLE
%  States:
%       x(1) = iL  inductor current [A]
%       x(2) = vC  capacitor voltage [V]
%
%  Measurements:
%       z(1) = measured iL
%       z(2) = measured vC
%
%  Model includes:
%       inductor resistance rL
%       diode voltage drop Vd
% ============================================================

%% Simulation parameters
Ts = 1e-5;              % Sampling time [s]
Tend = 0.05;            % Simulation time [s]
N = round(Tend / Ts);   % Number of samples
t = (0:N-1) * Ts;

%% Boost converter parameters
params.L   = 1e-3;      % Inductance [H]
params.C   = 470e-6;    % Capacitance [F]
params.R   = 20.0;      % Load resistance [ohm]
params.rL  = 0.15;      % Inductor resistance [ohm]
params.Vd  = 0.7;       % Diode voltage drop [V]
params.Vin = 12.0;      % Input voltage [V]

%% Duty cycle
duty = 0.45 * ones(1, N);

% Optional duty-cycle step
duty(t > 0.025) = 0.55;

%% True initial state
x_true = zeros(2, N);
x_true(:, 1) = [0.0; 12.0];

%% UKF initial estimate
x_hat = zeros(2, N);
x_hat(:, 1) = [0.2; 10.0];

%% Initial covariance
P = diag([1.0^2, 5.0^2]);

%% Process noise covariance
% Represents model uncertainty
Q = diag([0.02^2, 0.2^2]);

%% Measurement noise covariance
% Suppose current sensor std = 0.05 A
% Suppose voltage sensor std = 0.10 V
sigma_iL = 0.2;
sigma_vC = 0.5;

R_meas = diag([sigma_iL^2, sigma_vC^2]);

%% UKF parameters
ukf.alpha = 1e-3;
ukf.beta  = 2.0;
ukf.kappa = 0.0;

%% Storage
z_meas = zeros(2, N);
P_store = zeros(2, 2, N);
P_store(:, :, 1) = P;

%% Random seed for repeatability
rng(1);

%% ============================================================
%  MAIN SIMULATION LOOP
% ============================================================

for k = 2:N

    %% --------------------------------------------------------
    % Simulate true boost converter
    % ---------------------------------------------------------
    d = duty(k-1);

    x_true(:, k) = boost_discrete_model(x_true(:, k-1), d, params, Ts);

    %% Add artificial measurement noise
    noise = [sigma_iL * randn;
             sigma_vC * randn];

    z_meas(:, k) = x_true(:, k) + noise;

    %% --------------------------------------------------------
    % UKF prediction step
    % ---------------------------------------------------------
    [x_pred, P_pred, sigma_pred, Wm, Wc] = ukf_predict( ...
        x_hat(:, k-1), P, Q, d, params, Ts, ukf);

    %% --------------------------------------------------------
    % UKF update step
    %
    % Since both states are measured:
    %       h(x) = x
    % ---------------------------------------------------------
    [x_update, P_update] = ukf_update_full_state( ...
        x_pred, P_pred, sigma_pred, Wm, Wc, z_meas(:, k), R_meas);

    %% Store results
    x_hat(:, k) = x_update;
    P = P_update;
    P_store(:, :, k) = P;
end

%% ============================================================
%  PLOTS
% ============================================================

figure;

subplot(2,1,1);
plot(t, x_true(1,:), 'k', 'LineWidth', 1.5); hold on;
plot(t, z_meas(1,:), '.', 'Color', [0.7 0.7 0.7]);
plot(t, x_hat(1,:), 'r', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('i_L [A]');
legend('True', 'Measured', 'UKF Estimate');
title('Inductor Current Estimation');

subplot(2,1,2);
plot(t, x_true(2,:), 'k', 'LineWidth', 1.5); hold on;
plot(t, z_meas(2,:), '.', 'Color', [0.7 0.7 0.7]);
plot(t, x_hat(2,:), 'r', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('v_C [V]');
legend('True', 'Measured', 'UKF Estimate');
title('Capacitor Voltage Estimation');

figure;
plot(t, duty, 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Duty Cycle');
title('Duty Cycle');

%% ============================================================
%  LOCAL FUNCTIONS
% ============================================================

function x_next = boost_discrete_model(x, d, params, Ts)
    % Discrete-time boost converter averaged model.
    %
    % States:
    %   x(1) = iL
    %   x(2) = vC

    iL = x(1);
    vC = x(2);

    L   = params.L;
    C   = params.C;
    R   = params.R;
    rL  = params.rL;
    Vd  = params.Vd;
    Vin = params.Vin;

    % Continuous-time averaged dynamics
    diL = (Vin - rL * iL - (1 - d) * (vC + Vd)) / L;

    dvC = ((1 - d) * iL - vC / R) / C;

    % Forward Euler discretization
    iL_next = iL + Ts * diL;
    vC_next = vC + Ts * dvC;

    x_next = [iL_next; vC_next];
end

function [x_pred, P_pred, sigma_pred, Wm, Wc] = ukf_predict(x, P, Q, d, params, Ts, ukf)
    % UKF prediction step.

    n = length(x);

    alpha = ukf.alpha;
    beta  = ukf.beta;
    kappa = ukf.kappa;

    lambda = alpha^2 * (n + kappa) - n;

    %% Compute weights
    Wm = zeros(2*n + 1, 1);
    Wc = zeros(2*n + 1, 1);

    Wm(1) = lambda / (n + lambda);
    Wc(1) = lambda / (n + lambda) + (1 - alpha^2 + beta);

    for i = 2:(2*n + 1)
        Wm(i) = 1 / (2 * (n + lambda));
        Wc(i) = 1 / (2 * (n + lambda));
    end

    %% Generate sigma points
    sigma = generate_sigma_points(x, P, lambda);

    %% Propagate sigma points through nonlinear model
    sigma_pred = zeros(size(sigma));

    for i = 1:(2*n + 1)
        sigma_pred(:, i) = boost_discrete_model(sigma(:, i), d, params, Ts);
    end

    %% Predicted mean
    x_pred = zeros(n, 1);

    for i = 1:(2*n + 1)
        x_pred = x_pred + Wm(i) * sigma_pred(:, i);
    end

    %% Predicted covariance
    P_pred = zeros(n, n);

    for i = 1:(2*n + 1)
        dx = sigma_pred(:, i) - x_pred;
        P_pred = P_pred + Wc(i) * (dx * dx');
    end

    P_pred = P_pred + Q;

    %% Force symmetry for numerical stability
    P_pred = 0.5 * (P_pred + P_pred');
end

function [x_update, P_update] = ukf_update_full_state(x_pred, P_pred, sigma_pred, Wm, Wc, z, R_meas)
    % UKF measurement update for full-state measurement:
    %
    %   z = x + noise
    %
    % Therefore:
    %
    %   h(x) = x

    n = length(x_pred);
    m = length(z);

    num_sigma = 2*n + 1;

    %% Measurement sigma points
    % Since h(x) = x, the predicted measurement sigma points are
    % equal to the predicted state sigma points.
    Z_sigma = sigma_pred;

    %% Predicted measurement mean
    z_pred = zeros(m, 1);

    for i = 1:num_sigma
        z_pred = z_pred + Wm(i) * Z_sigma(:, i);
    end

    %% Innovation covariance S
    S = zeros(m, m);

    for i = 1:num_sigma
        dz = Z_sigma(:, i) - z_pred;
        S = S + Wc(i) * (dz * dz');
    end

    S = S + R_meas;

    %% Cross covariance Pxz
    Pxz = zeros(n, m);

    for i = 1:num_sigma
        dx = sigma_pred(:, i) - x_pred;
        dz = Z_sigma(:, i) - z_pred;

        Pxz = Pxz + Wc(i) * (dx * dz');
    end

    %% Kalman gain
    % Equivalent to:
    %   K = Pxz / S
    % but written in a numerically stable way.
    K = Pxz / S;

    %% Innovation
    y = z - z_pred;

    %% State update
    x_update = x_pred + K * y;

    %% Covariance update
    P_update = P_pred - K * S * K';

    %% Force symmetry
    P_update = 0.5 * (P_update + P_update');
end

function sigma = generate_sigma_points(x, P, lambda)
    % Generate UKF sigma points.

    n = length(x);

    sigma = zeros(n, 2*n + 1);

    sigma(:, 1) = x;

    %% Numerical stabilization
    P = 0.5 * (P + P');

    jitter = 1e-12;
    success = false;

    for attempt = 1:5
        [S, flag] = chol((n + lambda) * P + jitter * eye(n), 'lower');

        if flag == 0
            success = true;
            break;
        else
            jitter = jitter * 10;
        end
    end

    if ~success
        error('Cholesky decomposition failed. Covariance matrix is not positive definite.');
    end

    for i = 1:n
        sigma(:, i+1)   = x + S(:, i);
        sigma(:, i+1+n) = x - S(:, i);
    end
end