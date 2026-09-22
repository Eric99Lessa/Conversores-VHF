clear; clc; close all;

%% Load data
load('boost_data.mat')

% Expected variables inside boost_data.mat:
% IL_m    : measured inductor currents, size [n_phases x N]
% Vout_m  : measured output voltage,    size [1 x N] or [N x 1]
% Vin_m   : measured input voltage,     size [1 x N] or [N x 1]
% Ts      : sampling time

%% Boost Converter Parameters
L  = 80e-3;   % Inductance [H]
RL = 10e-3;    % Inductor series resistance [Ohm]
VD = 0.8;     % Diode forward voltage [V]
C  = 12e-3;   % Output capacitance [F]

n_phases = min(size(IL_m));

%% Input duty cycle
d = 0.5 * ones(n_phases, 1);   % per-phase duty cycle

%% Make measurement data row-consistent
Vout_m = Vout_m(:).';
Vin_m  = Vin_m(:).';

%% Simulation length from data
N = size(IL_m, 2);
t = (0:N-1) * Ts;

%% State / measurement dimensions
% x = [IL_1; ...; IL_n;  Vout;  Vin;  Iload]
% z = [IL_1; ...; IL_n;  Vout;  Vin ]

n_states = n_phases + 3;
n_meas   = n_phases + 2;

idx_IL    = 1:n_phases;
idx_Vout  = n_phases + 1;
idx_Vin   = n_phases + 2;
idx_Iload = n_phases + 3;

%% Measurement matrix H  (linear measurement model → no linearisation needed)
% z = H * x
H = [eye(n_phases),         zeros(n_phases,1), zeros(n_phases,1), zeros(n_phases,1);
     zeros(1,n_phases),     1,                 0,                 0;
     zeros(1,n_phases),     0,                 1,                 0];
% size: [n_meas x n_states]

%% Noise covariances

% Process noise  (tune q_Iload / q_Vin for faster tracking of those states)
q_IL    = 0.0001;
q_Vout  = 0.0001;
q_Iload = 0.001;
q_Vin   = 0.0001;

Q = diag([q_IL*ones(n_phases,1); q_Vout; q_Vin; q_Iload]);

% Measurement noise
sigma_iL   = 0.1;
sigma_Vout = 0.1;
sigma_Vin  = 0.1;

R_meas = diag([sigma_iL^2 * ones(n_phases,1); sigma_Vout^2; sigma_Vin^2]);

%% EKF initial estimate
x_pred = zeros(n_states, 1);
x_pred(idx_IL)    = IL_m(:, 1);
x_pred(idx_Vout)  = Vout_m(1);
x_pred(idx_Vin)   = Vin_m(1);
x_pred(idx_Iload) = sum(IL_m(:, 1));   % rough initial guess

P_pred = eye(n_states);   % initial state covariance

%% Measurements matrix  z = [IL_m; Vout_m; Vin_m]
z_meas = [IL_m; Vout_m; Vin_m];

%% Storage
x_hat_store  = zeros(n_states, N);
x_pred_store = zeros(n_states, N);
P_store      = zeros(n_states, n_states, N);

x_hat_store(:, 1) = x_pred;
P_store(:,:,  1)  = P_pred;

%% ── Main EKF Loop ────────────────────────────────────────────────────────
for k = 1:N-1
    %% ── UPDATE STEP ──────────────────────────────────────────────────
    [x_hat, P] = ekf_update(x_pred, P_pred, z_meas(:, k+1), H, R_meas);

    %% ── PREDICTION STEP ──────────────────────────────────────────────
    [x_pred, P_pred, F] = ekf_predict(x_hat, P, Q, d, L, C, RL, VD, Ts, n_phases);

    x_pred_store(:, k+1) = x_pred;    

    %% ── Store ───────────────────────────────────────────────────────────
    x_hat_store(:, k+1) = x_hat;
    P_store(:,:,  k+1)  = P;
end

%% ── Plots ────────────────────────────────────────────────────────────────
colors = get(groot,'DefaultAxesColorOrder');

%% Figure 1 – Phase inductor currents
figure('Name','Phase Currents');
for i = 1:n_phases
    subplot(n_phases, 1, i); hold on; grid on;
    plot(t, IL_m(i,:),                  'Color', colors(1,:), 'LineWidth', 1.2, 'DisplayName', 'Measured');
    plot(t, x_hat_store(idx_IL(i), :),  'Color', colors(2,:), 'LineWidth', 1.2, 'DisplayName', 'EKF estimate');
    ylabel(sprintf('I_{L%d} (A)', i));
    legend('Location','best');
    if i == 1; title('Inductor Currents'); end
end
xlabel('Time (s)');

%% Figure 2 – Output voltage
figure('Name','Output Voltage'); hold on; grid on;
plot(t, Vout_m,                    'Color', colors(1,:), 'LineWidth', 1.2, 'DisplayName', 'Measured');
plot(t, x_hat_store(idx_Vout, :),  'Color', colors(2,:), 'LineWidth', 1.2, 'DisplayName', 'EKF estimate');
ylabel('V_{out} (V)'); xlabel('Time (s)');
title('Output Voltage'); legend('Location','best');

%% Figure 3 – Input voltage
figure('Name','Input Voltage'); hold on; grid on;
plot(t, Vin_m,                    'Color', colors(1,:), 'LineWidth', 1.2, 'DisplayName', 'Measured');
plot(t, x_hat_store(idx_Vin, :),  'Color', colors(2,:), 'LineWidth', 1.2, 'DisplayName', 'EKF estimate');
ylabel('V_{in} (V)'); xlabel('Time (s)');
title('Input Voltage'); legend('Location','best');

%% Figure 4 – Estimated load current (unmeasured state)
figure('Name','Load Current Estimate'); hold on; grid on;
plot(t, x_hat_store(idx_Iload, :), 'Color', colors(3,:), 'LineWidth', 1.5, 'DisplayName', 'EKF estimate');
ylabel('I_{load} (A)'); xlabel('Time (s)');
title('Load Current (Unmeasured State)'); legend('Location','best');

%% Figure 5 – Innovations (residuals)
innovation = z_meas - H * x_hat_store;
meas_labels = [arrayfun(@(i) sprintf('I_{L%d}',i), 1:n_phases, 'UniformOutput',false), ...
               {'V_{out}', 'V_{in}'}];
figure('Name','Innovations (Residuals)');
for i = 1:n_meas
    subplot(n_meas, 1, i); hold on; grid on;
    plot(t, innovation(i,:), 'Color', colors(mod(i-1,7)+1,:), 'LineWidth', 1.0);
    yline(0,'k--');
    ylabel(meas_labels{i});
    if i == 1; title('Innovations  (z_{meas} - H \hat{x})'); end
end
xlabel('Time (s)');

%% Figure 6 – State estimation std dev  (√P_ii)
P_diag = zeros(n_states, N);
for k = 1:N
    P_diag(:,k) = diag(P_store(:,:,k));
end
P_diag = sqrt(max(0, P_diag));
state_labels = [arrayfun(@(i) sprintf('I_{L%d}',i), 1:n_phases, 'UniformOutput',false), ...
                {'V_{out}', 'V_{in}', 'I_{load}'}];
figure('Name','Estimation Uncertainty (std)');
for i = 1:n_states
    subplot(n_states, 1, i); hold on; grid on;
    plot(t, P_diag(i,:), 'Color', colors(mod(i-1,7)+1,:), 'LineWidth', 1.2);
    ylabel(['\sigma(' state_labels{i} ')']);
    if i == 1; title('State Estimation Std Dev  (\surdP_{ii})'); end
end
xlabel('Time (s)');


%% ════════════════════════════════════════════════════════════════════════
%%  EKF FUNCTIONS
%% ════════════════════════════════════════════════════════════════════════

function [x_pred, P_pred, F] = ekf_predict(x, P, Q, d, L, C, RL, VD, Ts, n_phases)
% EKF_PREDICT  EKF prediction step for a multi-phase boost converter.
%
%  State:  x = [IL_1; ...; IL_n;  Vout;  Vin;  Iload]
%
%  Averaged continuous-time model (per phase i):
%    L  * dIL_i/dt = Vin - RL*IL_i - (1-d_i)*(Vout + VD)
%    C  * dVout/dt = sum_i[ (1-d_i)*IL_i ] - Iload
%    dVin/dt       = 0   (modelled as random walk via Q)
%    dIload/dt     = 0   (modelled as random walk via Q)
%
%  Discretised with forward Euler at step Ts.
%
%  Inputs
%    x        Current state estimate          [n_states x 1]
%    P        Current covariance              [n_states x n_states]
%    Q        Process noise covariance        [n_states x n_states]
%    d        Per-phase duty cycle            [n_phases  x 1]
%    L,C,RL,VD,Ts  Converter parameters / sample time
%    n_phases Number of interleaved phases
%
%  Outputs
%    x_pred   Predicted state                 [n_states x 1]
%    P_pred   Predicted covariance            [n_states x n_states]
%    F        State Jacobian at x             [n_states x n_states]

    n_states = n_phases + 3;
    q = 1 - d;   % complementary duty cycle, [n_phases x 1]

    IL    = x(1:n_phases);
    Vout  = x(n_phases + 1);
    Vin   = x(n_phases + 2);
    Iload = x(n_phases + 3);

    % ── Nonlinear propagation  f(x) ──────────────────────────────────────
    dIL   = (Vin - RL*IL - q.*(Vout + VD)) / L;   % [n_phases x 1]
    dVout = (q' * IL - Iload) / C;                 % scalar

    x_pred = zeros(n_states, 1);
    x_pred(1:n_phases)   = IL    + Ts * dIL;
    x_pred(n_phases + 1) = Vout  + Ts * dVout;
    x_pred(n_phases + 2) = Vin;                    % random walk
    x_pred(n_phases + 3) = Iload;                  % random walk

    % ── Jacobian  F = ∂f/∂x ─────────────────────────────────────────────
    %
    %  Block structure (phase index i, columns ordered as state vector):
    %
    %         IL_1 … IL_n   Vout          Vin     Iload
    %  IL_i [ ...  ...  ...  -Ts*q_i/L    Ts/L     0   ]  diagonal: 1-Ts*RL/L
    %  Vout [ Ts*q_1/C … Ts*q_n/C  1     0        -Ts/C]
    %  Vin  [    0    …    0        0     1         0   ]
    %  Iload[    0    …    0        0     0         1   ]

    F = eye(n_states);

    % Rows for IL_i  (i = 1…n_phases)
    for i = 1:n_phases
        F(i, i)            = 1 - Ts * RL / L;    % ∂IL_i/∂IL_i
        F(i, n_phases + 1) = -Ts * q(i) / L;     % ∂IL_i/∂Vout
        F(i, n_phases + 2) =  Ts / L;            % ∂IL_i/∂Vin
    end

    % Row for Vout
    for i = 1:n_phases
        F(n_phases+1, i) = Ts * q(i) / C;        % ∂Vout/∂IL_i
    end
    F(n_phases+1, n_phases+3) = -Ts / C;          % ∂Vout/∂Iload

    % Rows for Vin and Iload are already identity (random-walk states)

    % ── Covariance prediction ─────────────────────────────────────────────
    P_pred = F * P * F' + Q;
end


function [x_upd, P_upd] = ekf_update(x_pred, P_pred, z, H, R)
% EKF_UPDATE  EKF measurement update step.
%
%  The measurement model is LINEAR:  z = H * x + v
%  so the EKF update is exact (identical to the standard KF update).
%
%  Inputs
%    x_pred   Predicted state                       [n_states x 1]
%    P_pred   Predicted covariance                  [n_states x n_states]
%    z        Measurement vector                    [n_meas   x 1]
%    H        Measurement matrix                    [n_meas   x n_states]
%    R        Measurement noise covariance          [n_meas   x n_meas]
%
%  Outputs
%    x_upd    Updated (posterior) state             [n_states x 1]
%    P_upd    Updated (posterior) covariance        [n_states x n_states]

    % Innovation
    y = z - H * x_pred;

    % Innovation covariance
    S = H * P_pred * H' + R;

    % Kalman gain
    K = P_pred * H' / S;

    % State update
    x_upd = x_pred + K * y;

    % Covariance update – Joseph form for numerical stability
    I     = eye(size(P_pred));
    IKH   = I - K * H;
    P_upd = IKH * P_pred * IKH' + K * R * K';
end
