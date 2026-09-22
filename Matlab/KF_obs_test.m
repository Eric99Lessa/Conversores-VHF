clear; clc; close all;

%% Load data
load('boost_data.mat')

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

%% State Definition
% x = [IL_1; IL_2; ...; IL_n; Vout; Vin; 
%      IL_dot_1; IL_dot_2; ...; IL_dot_n; Vout_dot; Vin_dot]

order_approx = 0;
n_meas   = n_phases + 2;
n_states = (order_approx + 1)*n_meas;
n_sigma  = 2*n_states + 1;

idx_IL    = 1:n_phases;
idx_Vout  = n_phases + 1;
idx_Vin   = n_phases + 2;

%% Filter Matrixes
% Transition
if order_approx == 0
    F = eye(n_states);
elseif order_approx == 1
    F = eye(n_states) + ...
        diag(Ts*ones(n_states - n_meas, 1), n_meas);
elseif order_approx == 2
    F = eye(n_states) + ...
        Ts*diag(ones(n_states - n_meas, 1), n_meas) + ...
        (Ts^2/2)*diag(ones(n_states - 2*n_meas, 1), 2*n_meas);
else
    F = zeros(n_states);
end

% Measurement
H = [eye(n_meas) zeros(n_meas, n_states - n_meas)];

% Uncertainty
P_pred = eye(n_states);

% Measurement noise
sigma_iL   = 0.1;
sigma_Vout = 0.1;
sigma_Vin  = 0.1;

R = diag([
    (sigma_iL^2)*ones(n_phases, 1);
    sigma_Vout^2;
    sigma_Vin^2
]);

% Process
% Individual q values for each of the 5 variables
q_iL = 1e-3;
q_Vout = 1e-3;
q_Vin = 1e-3;

if order_approx == 0
    Q = diag([q_iL*ones(n_phases, 1); q_Vout; q_Vin]);
elseif order_approx == 1
    q_iL_dot = 1e-4;
    q_Vout_dot = 1e-4;
    q_Vin_dot = 1e-4;
    Q = diag([q_iL*ones(n_phases, 1); q_Vout; q_Vin; ...
         q_iL_dot*ones(n_phases, 1); q_Vout_dot; q_Vin_dot]);
elseif order_approx == 2
    q_iL_ddot = 1e-1;
    q_Vout_ddot = 1e-1;
    q_Vin_ddot = 1e-1;
    Q = diag([q_iL*ones(n_phases, 1); q_Vout; q_Vin; ...
         q_iL_dot*ones(n_phases, 1); q_Vout_dot; q_Vin_dot; ...
         q_iL_ddot*ones(n_phases, 1); q_Vout_ddot; q_Vin_ddot]);
end

%% Measurements
% y = [IL_m; Vout_m; Vin_m]
y_meas = [
    IL_m;
    Vout_m;
    Vin_m
];

%% Initial Estimate
x_pred = zeros(n_states, 1);

% Initialize measured states from first samples
x_pred(idx_IL, 1)   = IL_m(:, 1);
x_pred(idx_Vout, 1) = Vout_m(1);
x_pred(idx_Vin, 1)  = Vin_m(1);

%% Storage
% Simulation length from data
N = size(IL_m, 2);
t = (0:N-1)*Ts;

x_hat_store = zeros(n_states, N);
x_pred_store = zeros(n_states, N);
P_store = zeros(n_states, n_states, N);
P_store(:, :, 1) = P_pred;

%% Main UKF Loop
for k = 1:N-1
    %% Update step
    [x_hat, P] = KF_update(x_pred, P_pred, y_meas(:, k), H, R);
    
    %% Prediction step
    [x_pred, P_pred] = KF_predict(x_hat, P, F, Q);

    %% Store
    x_hat_store(:, k) = x_hat;
    x_pred_store(:, k+1) = x_pred;
    P_store(:, :, k+1) = P;
end
x_hat_store(:, end) = x_hat_store(:, end-1);

%% Figure 1 – Phase inductor currents
figure('Name', 'Phase Currents');
for i = 1:n_phases
    subplot(n_phases, 1, i); hold on; grid on;
    plot(t, IL_m(i,:), 'LineWidth', 2, 'DisplayName', 'Measured');
    plot(t, x_hat_store(idx_IL(i),:), 'LineWidth', 2, 'DisplayName', 'UKF estimate');
    ylabel(sprintf('I_{L%d} (A)', i));
    legend('Location','best');
    if i == 1; title('Inductor Currents'); end
end
xlabel('Time (s)');

%% Figure 2 – Output voltage
figure('Name', 'Output Voltage'); hold on; grid on;
plot(t, Vout_m, 'LineWidth', 2, 'DisplayName', 'Measured');
plot(t, x_hat_store(idx_Vout,:), 'LineWidth', 2, 'DisplayName', 'UKF estimate');
ylabel('V_{out} (V)'); xlabel('Time (s)');
title('Output Voltage'); legend('Location','best');

%% Figure 3 – Input voltage
figure('Name', 'Input Voltage'); hold on; grid on;
plot(t, Vin_m, 'LineWidth', 2, 'DisplayName', 'Measured');
plot(t, x_hat_store(idx_Vin,:), 'LineWidth', 2, 'DisplayName', 'UKF estimate');
ylabel('V_{in} (V)'); xlabel('Time (s)');
title('Input Voltage'); legend('Location','best');

if order_approx > 0
    idx_IL_dot = idx_IL + n_meas;
    idx_Vout_dot = idx_Vout + n_meas;
    idx_Vin_dot = idx_Vin + n_meas;

    %% Figure 4 – Phase inductor currents derivatives
    figure('Name', 'Phase Currents');
    for i = 1:n_phases
        subplot(n_phases, 1, i); hold on; grid on;
        plot(t, x_hat_store(idx_IL_dot(i),:), 'LineWidth', 2, 'DisplayName', 'UKF estimate');
        ylabel(sprintf('I_{L%d} (A)', i));
        legend('Location','best');
        if i == 1; title('Inductor Currents Derivative'); end
    end
    xlabel('Time (s)');
    
    %% Figure 5 – Output voltage derivative
    figure('Name', 'Output Voltage'); hold on; grid on;
    plot(t, x_hat_store(idx_Vout_dot,:), 'LineWidth', 2, 'DisplayName', 'UKF estimate');
    ylabel('V_{out} (V)'); xlabel('Time (s)');
    title('Output Voltage Derivative'); legend('Location','best');
    
    %% Figure 6 – Input voltage derivative
    figure('Name', 'Input Voltage'); hold on; grid on;
    plot(t, x_hat_store(idx_Vin_dot,:), 'LineWidth', 2, 'DisplayName', 'UKF estimate');
    ylabel('V_{in} (V)'); xlabel('Time (s)');
    title('Input Voltage Derivative'); legend('Location','best');
end

if order_approx > 1
    idx_IL_ddot = idx_IL + 2*n_meas;
    idx_Vout_ddot = idx_Vout + 2*n_meas;
    idx_Vin_ddot = idx_Vin + 2*n_meas;

    %% Figure 7 – Phase inductor currents derivatives
    figure('Name', 'Phase Currents');
    for i = 1:n_phases
        subplot(n_phases, 1, i); hold on; grid on;
        plot(t, x_hat_store(idx_IL_ddot(i),:), 'LineWidth', 2, 'DisplayName', 'UKF estimate');
        ylabel(sprintf('I_{L%d} (A)', i));
        legend('Location','best');
        if i == 1; title('Inductor Currents Second Derivative'); end
    end
    xlabel('Time (s)');
    
    %% Figure 8 – Output voltage derivative
    figure('Name', 'Output Voltage'); hold on; grid on;
    plot(t, x_hat_store(idx_Vout_ddot,:), 'LineWidth', 2, 'DisplayName', 'UKF estimate');
    ylabel('V_{out} (V)'); xlabel('Time (s)');
    title('Output Voltage Second Derivative'); legend('Location','best');
    
    %% Figure 9 – Input voltage derivative
    figure('Name', 'Input Voltage'); hold on; grid on;
    plot(t, x_hat_store(idx_Vin_ddot,:), 'LineWidth', 2, 'DisplayName', 'UKF estimate');
    ylabel('V_{in} (V)'); xlabel('Time (s)');
    title('Input Voltage Second Derivative'); legend('Location','best');
end

%% Predict Function
function [x_pred, P_pred] = KF_predict(x, P, F, Q)
    x_pred = F * x;
    P_pred = F * P * F' + Q;
end

%% Update Function
function [x_upd, P_upd] = KF_update(x_pred, P_pred, z, H, R)
    % Innovation (measurement residual)
    y = z - H * x_pred;

    % Innovation covariance
    S = H * P_pred * H' + R;

    % Kalman gain
    K = P_pred * H' / S;

    % State update
    x_upd = x_pred + K * y;

    % Covariance update (Joseph form for numerical stability)
    I = eye(size(P_pred));
    P_upd = (I - K * H) * P_pred * (I - K * H)' + K * R * K';
end