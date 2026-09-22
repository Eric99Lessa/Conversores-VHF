clear; clc; close all;

%% Load data
load('boost_data.mat')

% Expected variables inside boost_data.mat:
% IL_m    : measured phase currents, size [n_phases x N]
% Vout_m  : measured output voltage, size [1 x N] or [N x 1]
% Vin_m   : measured input voltage, size [1 x N] or [N x 1]
% Ts      : sampling time

%% Boost Converter Parameters
L  = 80e-3;       % Inductance [H]
RL = 10e3;        % Inductor series resistance [Ohm]
VD = 0.8;         % Diode voltage drop [V]
C  = 12e-3;       % Output capacitance [F]

n_phases = min(size(IL_m));

%% Input duty cycle
d = 0.5*ones(n_phases, 1);

%% Make measurement data row-consistent
Vout_m = Vout_m(:).';
Vin_m  = Vin_m(:).';

%% Simulation length from data
N = size(IL_m, 2);
t = (0:N-1)*Ts;

%% State Definition
% x = [IL_1; IL_2; ...; IL_n; Vout; Vin; I_load]

n_states = n_phases + 3;
n_meas   = n_phases + 2;
n_sigma  = 2*n_states + 1;

idx_IL    = 1:n_phases;
idx_Vout  = n_phases + 1;
idx_Vin   = n_phases + 2;
idx_Iload = n_phases + 3;

%% UKF Parameters
alpha = 1e-3;
beta  = 2;
kappa = 0;

lambda = alpha^2*(n_states + kappa) - n_states;

Wm = zeros(n_sigma, 1);
Wc = zeros(n_sigma, 1);

Wm(1) = lambda/(n_states + lambda);
Wc(1) = lambda/(n_states + lambda) + (1 - alpha^2 + beta);

for i = 2:n_sigma
    Wm(i) = 1/(2*(n_states + lambda));
    Wc(i) = 1/(2*(n_states + lambda));
end

%% UKF Initial Estimate
x_pred = zeros(n_states, 1);

% Initialize measured states from first samples
x_pred(idx_IL, 1)   = IL_m(:, 1);
x_pred(idx_Vout, 1) = Vout_m(1);
x_pred(idx_Vin, 1)  = Vin_m(1);

% Initial load current estimate
x_pred(idx_Iload, 1) = sum(IL_m(:, 1));

%% Covariance Matrices

% Initial covariance
P_pred = eye(n_states);

% Process noise
% Increase q_Iload if load changes faster.
% Increase q_Vin if input voltage changes faster.
q_IL    = 1e-1;
q_Vout  = 1e-1;
q_Iload = 0;
q_Vin   = 0;

Q = diag([
    q_IL*ones(n_phases, 1);
    q_Vout;
    q_Vin;
    q_Iload
]);

Xsigma_pred = generateSigmaPoints(x_pred(:, 1), P_pred, lambda);

% Measurement noise
sigma_iL   = 0.1;
sigma_Vout = 0.1;
sigma_Vin  = 0.1;

R_meas = diag([
    sigma_iL^2*ones(n_phases, 1);
    sigma_Vout^2;
    sigma_Vin^2
]);

%% Measurements
% y = [IL_m; Vout_m; Vin_m]
y_meas = [
    IL_m;
    Vout_m;
    Vin_m
];

%% Storage
x_hat_store = zeros(n_states, N);
x_pred_store = zeros(n_states, N);
P_store = zeros(n_states, n_states, N);
P_store(:, :, 1) = P_pred;

%% Main UKF Loop
for k = 1:N-1
    %% Update step
    [x_hat, P] = update_step( ...
        x_pred, ...
        P_pred, ...
        Xsigma_pred, ...
        y_meas(:, k+1), ...
        R_meas, ...
        Wm, ...
        Wc, ...
        n_phases);
    
    %% Prediction step
    [x_pred, P_pred, Xsigma_pred] = predict_step( ...
        x_hat, ...
        P, ...
        Q, ...
        d, ...
        L, ...
        C, ...
        RL, ...
        VD, ...
        Ts, ...
        n_phases, ...
        lambda, ...
        Wm, ...
        Wc);

    %% Store
    x_hat_store(:, k+1) = x_hat;
    x_pred_store(:, k+1) = x_pred;
    P_store(:, :, k+1) = P;
end

%% ── Plots ────────────────────────────────────────────────────────────────
colors = get(groot, 'DefaultAxesColorOrder');  % MATLAB default palette

%% Figure 1 – Phase inductor currents
figure('Name', 'Phase Currents');
for i = 1:n_phases
    subplot(n_phases, 1, i); hold on; grid on;
    plot(t, IL_m(i,:),          'Color', colors(1,:), 'LineWidth', 1.2, 'DisplayName', 'Measured');
    plot(t, x_hat_store(idx_IL(i),:), 'Color', colors(2,:), 'LineWidth', 1.2, 'DisplayName', 'UKF estimate');
    ylabel(sprintf('I_{L%d} (A)', i));
    legend('Location','best');
    if i == 1; title('Inductor Currents'); end
end
xlabel('Time (s)');

%% Figure 2 – Output voltage
figure('Name', 'Output Voltage'); hold on; grid on;
plot(t, Vout_m,              'Color', colors(1,:), 'LineWidth', 1.2, 'DisplayName', 'Measured');
plot(t, x_hat_store(idx_Vout,:),   'Color', colors(2,:), 'LineWidth', 1.2, 'DisplayName', 'UKF estimate');
ylabel('V_{out} (V)'); xlabel('Time (s)');
title('Output Voltage'); legend('Location','best');

%% Figure 3 – Input voltage
figure('Name', 'Input Voltage'); hold on; grid on;
plot(t, Vin_m,              'Color', colors(1,:), 'LineWidth', 1.2, 'DisplayName', 'Measured');
plot(t, x_hat_store(idx_Vin,:),   'Color', colors(2,:), 'LineWidth', 1.2, 'DisplayName', 'UKF estimate');
ylabel('V_{in} (V)'); xlabel('Time (s)');
title('Input Voltage'); legend('Location','best');

%% Figure 4 – Estimated load current (unmeasured state)
figure('Name', 'Load Current Estimate'); hold on; grid on;
plot(t, x_hat_store(idx_Iload,:), 'Color', colors(3,:), 'LineWidth', 1.5, 'DisplayName', 'UKF estimate');
ylabel('I_{load} (A)'); xlabel('Time (s)');
title('Load Current (Unmeasured State)'); legend('Location','best');

%% Figure 5 – Innovation (residuals) for measured outputs
innovation = y_meas - [x_hat(idx_IL,:); x_hat(idx_Vout,:); x_hat(idx_Vin,:)];
meas_labels = [arrayfun(@(i) sprintf('I_{L%d}',i), 1:n_phases, 'UniformOutput',false), ...
               {'V_{out}', 'V_{in}'}];
figure('Name', 'Innovations (Residuals)');
for i = 1:n_meas
    subplot(n_meas, 1, i); hold on; grid on;
    plot(t, innovation(i,:), 'Color', colors(mod(i-1,7)+1,:), 'LineWidth', 1.0);
    yline(0, 'k--');
    ylabel(meas_labels{i});
    if i == 1; title('Innovations (y_{meas} - y_{hat})'); end
end
xlabel('Time (s)');

%% Figure 6 – State estimation covariance (diagonal)
figure('Name', 'Estimation Uncertainty (std)');
P_diag = zeros(n_states, N);
for k = 1:N
    P_diag(:,k) = diag(P_store(:,:,k));
end
P_diag = sqrt(max(0, P_diag));
state_labels = [arrayfun(@(i) sprintf('I_{L%d}',i), 1:n_phases, 'UniformOutput',false), ...
                {'V_{out}', 'V_{in}', 'I_{load}'}];
for i = 1:n_states
    subplot(n_states, 1, i); hold on; grid on;
    plot(t, P_diag(i,:), 'Color', colors(mod(i-1,7)+1,:), 'LineWidth', 1.2);
    ylabel(['\sigma(' state_labels{i} ')']);
    if i == 1; title('State Estimation Std Dev (\surd P_{ii})'); end
end
xlabel('Time (s)');