clear; clc; close all;

%% Load data
load('boost_data.mat')

% Expected variables inside boost_data.mat:
% IL_m    : measured inductor currents, size [n_phases x N]
% Vout_m  : measured output voltage,    size [1 x N] or [N x 1]
% Vin_m   : measured input voltage,     size [1 x N] or [N x 1]
% Ts      : sampling time

%% Boost Converter Parameters
% Only C is used in the pseudo-measurement (soft node constraint).
% L, RL, VD are NOT used anywhere in the filter.
C  = 12e-3;   % Output capacitance [F]

n_phases = min(size(IL_m));

%% Input duty cycle
d = 0.5 * ones(n_phases, 1);   % per-phase duty cycle [n_phases x 1]

%% Make measurement data row-consistent
Vout_m = Vout_m(:).';
Vin_m  = Vin_m(:).';

%% Simulation length
N = size(IL_m, 2);
t = (0:N-1) * Ts;

%% ── State Definition ─────────────────────────────────────────────────────
%
%  x = [ IL_1; ...; IL_n;      ← phase currents         (n_phases)
%         dIL_1; ...; dIL_n;   ← current derivatives    (n_phases)
%         Vout;                 ← output voltage         (1)
%         dVout;                ← voltage derivative     (1)
%         Vin;                  ← input voltage          (1)
%         dVin;                 ← input derivative       (1)
%         Iload ]               ← load current           (1)   ← independent random walk
%
%  Prediction (all kinematic, zero physical parameters):
%    IL_i(k+1)    = IL_i   + Ts·dIL_i        dIL_i(k+1)   = dIL_i
%    Vout(k+1)    = Vout   + Ts·dVout         dVout(k+1)   = dVout
%    Vin(k+1)     = Vin    + Ts·dVin          dVin(k+1)    = dVin
%    Iload(k+1)   = Iload                    ← pure random walk (smooth)
%
%  Measurement vector  z = [IL_meas; Vout_meas; Vin_meas; z_node]
%    z_node  = 0  (pseudo-measurement of the node residual)
%
%  Measurement model  z = H·x + v :
%    [IL]     = [I   0   0  0  0  0   0 ] · x + v_IL
%    [Vout]   = [0   0   1  0  0  0   0 ] · x + v_Vout
%    [Vin]    = [0   0   0  0  1  0   0 ] · x + v_Vin
%    [0]      = [q'  0   0  -C  0  0  -1] · x + v_node
%               └─ soft node equation: Σq_i·IL_i - C·dVout - Iload = 0
%
%  sigma_Iload (noise on the pseudo-measurement) is the SOLE smoothing knob:
%    small  → Iload tightly follows the node equation  (less smooth, faster)
%    large  → Iload is smoothed away from instantaneous node value (smoother)

n_states = 2*n_phases + 5;
n_meas   = n_phases + 3;       % IL, Vout, Vin  +  1 pseudo-measurement

idx_IL    = 1          : n_phases;
idx_Vout = n_phases + 1;
idx_Vin = n_phases + 2;
idx_dIL   = (n_phases+3):(2*n_phases + 2);
idx_dVout = 2*n_phases + 3;
idx_dVin  = 2*n_phases + 4;
idx_Iload = 2*n_phases + 5;

%% ── Measurement matrix H ─────────────────────────────────────────────────
%
%  Rows: [IL_1 ... IL_n | Vout | Vin | node_residual]
%  Cols: [IL  | dIL | Vout | dVout | Vin | dVin | Iload]

H = zeros(n_meas, n_states);

% Physical measurements
for i = 1:n_phases
    H(i, idx_IL(i)) = 1;           % measure IL_i
end
H(n_phases+1, idx_Vout) = 1;       % measure Vout
H(n_phases+2, idx_Vin)  = 1;       % measure Vin

% Pseudo-measurement row:  Σq_i·IL_i - C·dVout - Iload = 0
q = 1 - d;
for i = 1:n_phases
    H(n_phases+3, idx_IL(i)) = q(i);   % ∂node/∂IL_i
end
H(n_phases+3, idx_dVout) = -C;         % ∂node/∂dVout
H(n_phases+3, idx_Iload) = -1;         % ∂node/∂Iload

%% ── Noise Covariances ────────────────────────────────────────────────────

% Process noise
q_IL    = 1e-4;
q_dIL   = 1e-1;   % main knob: derivative tracking speed
q_Vout  = 1e-4;
q_dVout = 1e-1;
q_Vin   = 1e-4;
q_dVin  = 1e-1;
q_Iload = 1e-2;   % random-walk noise on Iload; tune for tracking speed

Q = diag([
    q_IL    * ones(n_phases, 1);
    q_dIL   * ones(n_phases, 1);
    q_Vout;
    q_dVout;
    q_Vin;
    q_dVin;
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

%% ── EKF Initial Estimate ─────────────────────────────────────────────────
x_hat = zeros(n_states, 1);
x_hat(idx_IL)    = IL_m(:, 1);
x_hat(idx_dIL)   = zeros(n_phases, 1);
x_hat(idx_Vout)  = Vout_m(1);
x_hat(idx_dVout) = 0;
x_hat(idx_Vin)   = Vin_m(1);
x_hat(idx_dVin)  = 0;
x_hat(idx_Iload) = q' * IL_m(:,1);   % node equation at t=0 as warm start

P = blkdiag(eye(n_phases),       ...  % IL
            10*eye(n_phases),    ...  % dIL
            1, 10, 1, 10,        ...  % Vout, dVout, Vin, dVin
            1);                       % Iload

%% ── Measurement vector (real + pseudo) ───────────────────────────────────
%
%  The pseudo-measurement target is always 0  (residual of node equation).
%  H already encodes the full node equation, so z_node = 0 always.

z_meas = [
    IL_m;
    Vout_m;
    Vin_m;
    zeros(1, N)    % pseudo-measurement: node residual target = 0
];

%% ── Storage ──────────────────────────────────────────────────────────────
x_hat_store  = zeros(n_states, N);
x_pred_store = zeros(n_states, N);
P_store      = zeros(n_states, n_states, N);

x_hat_store(:, 1) = x_hat;
P_store(:,:,  1)  = P;

%% ── Main EKF Loop ────────────────────────────────────────────────────────
for k = 1:N-1

    % Prediction (fully kinematic — no physical parameters)
    [x_pred, P_pred] = ekf_predict(x_hat, P, Q, Ts, n_phases, ...
                                   idx_IL, idx_dIL, idx_Vout, idx_dVout, ...
                                   idx_Vin, idx_dVin, idx_Iload);

    x_pred_store(:, k+1) = x_pred;

    % Update (real measurements + soft node constraint)
    [x_hat, P] = ekf_update(x_pred, P_pred, z_meas(:, k+1), H, R_meas);

    x_hat_store(:, k+1) = x_hat;
    P_store(:,:,  k+1)  = P;
end

%% ── Plots ────────────────────────────────────────────────────────────────
colors = get(groot,'DefaultAxesColorOrder');

%% Figure 1 – Phase inductor currents
figure('Name','Phase Currents');
for i = 1:n_phases
    subplot(n_phases, 1, i); hold on; grid on;
    plot(t, IL_m(i,:),                'Color', colors(1,:), 'LineWidth', 1.2, 'DisplayName', 'Measured');
    plot(t, x_hat_store(idx_IL(i),:), 'Color', colors(2,:), 'LineWidth', 1.2, 'DisplayName', 'EKF estimate');
    ylabel(sprintf('I_{L%d} (A)', i));
    legend('Location','best');
    if i == 1; title('Inductor Currents'); end
end
xlabel('Time (s)');

%% Figure 2 – Inductor current derivatives
figure('Name','Inductor Current Derivatives');
for i = 1:n_phases
    subplot(n_phases, 1, i); hold on; grid on;
    plot(t, x_hat_store(idx_dIL(i),:), 'Color', colors(2,:), 'LineWidth', 1.2, 'DisplayName', 'EKF dIL/dt');
    ylabel(sprintf('dI_{L%d}/dt (A/s)', i));
    legend('Location','best');
    if i == 1; title('Estimated Inductor Current Derivatives'); end
end
xlabel('Time (s)');

%% Figure 3 – Output voltage + derivative
figure('Name','Output Voltage');
subplot(2,1,1); hold on; grid on;
plot(t, Vout_m,                  'Color', colors(1,:), 'LineWidth', 1.2, 'DisplayName', 'Measured');
plot(t, x_hat_store(idx_Vout,:), 'Color', colors(2,:), 'LineWidth', 1.2, 'DisplayName', 'EKF estimate');
ylabel('V_{out} (V)'); title('Output Voltage'); legend('Location','best');

subplot(2,1,2); hold on; grid on;
plot(t, x_hat_store(idx_dVout,:), 'Color', colors(3,:), 'LineWidth', 1.2, 'DisplayName', 'EKF dVout/dt');
ylabel('dV_{out}/dt (V/s)'); title('Output Voltage Derivative'); legend('Location','best');
xlabel('Time (s)');

%% Figure 4 – Input voltage + derivative
figure('Name','Input Voltage');
subplot(2,1,1); hold on; grid on;
plot(t, Vin_m,                  'Color', colors(1,:), 'LineWidth', 1.2, 'DisplayName', 'Measured');
plot(t, x_hat_store(idx_Vin,:), 'Color', colors(2,:), 'LineWidth', 1.2, 'DisplayName', 'EKF estimate');
ylabel('V_{in} (V)'); title('Input Voltage'); legend('Location','best');

subplot(2,1,2); hold on; grid on;
plot(t, x_hat_store(idx_dVin,:), 'Color', colors(3,:), 'LineWidth', 1.2, 'DisplayName', 'EKF dVin/dt');
ylabel('dV_{in}/dt (V/s)'); title('Input Voltage Derivative'); legend('Location','best');
xlabel('Time (s)');

%% Figure 5 – Load current (smooth independent state)
figure('Name','Load Current'); hold on; grid on;
% Reference: instantaneous algebraic node value (pre-filter)
Iload_node = q' * IL_m - C * ([diff(Vout_m), 0] / Ts);
plot(t, Iload_node,                   'Color', colors(1,:), 'LineWidth', 0.8, ...
     'LineStyle','--', 'DisplayName', 'Node eq. (raw)');
plot(t, x_hat_store(idx_Iload,:),     'Color', colors(2,:), 'LineWidth', 1.5, ...
     'DisplayName', 'EKF estimate (smooth)');
ylabel('I_{load} (A)'); xlabel('Time (s)');
title('Load Current  —  Independent State + Soft Node Constraint');
legend('Location','best');

%% Figure 6 – Innovations (all rows including pseudo-measurement)
innovation = z_meas - H * x_hat_store;
meas_labels = [arrayfun(@(i) sprintf('I_{L%d}',i), 1:n_phases, 'UniformOutput',false), ...
               {'V_{out}', 'V_{in}', 'Node residual'}];
figure('Name','Innovations');
for i = 1:n_meas
    subplot(n_meas, 1, i); hold on; grid on;
    plot(t, innovation(i,:), 'Color', colors(mod(i-1,7)+1,:), 'LineWidth', 1.0);
    yline(0,'k--');
    ylabel(meas_labels{i});
    if i == 1; title('Innovations  (z - H\hat{x})'); end
end
xlabel('Time (s)');

%% Figure 7 – Estimation std dev
P_diag = zeros(n_states, N);
for k = 1:N
    P_diag(:,k) = diag(P_store(:,:,k));
end
P_diag = sqrt(max(0, P_diag));
state_labels = [arrayfun(@(i) sprintf('I_{L%d}',i),    1:n_phases, 'UniformOutput',false), ...
                arrayfun(@(i) sprintf('dI_{L%d}/dt',i), 1:n_phases, 'UniformOutput',false), ...
                {'V_{out}', 'dV_{out}/dt', 'V_{in}', 'dV_{in}/dt', 'I_{load}'}];
figure('Name','Estimation Uncertainty');
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

function [x_pred, P_pred] = ekf_predict(x, P, Q, Ts, n_phases, ...
                                         idx_IL, idx_dIL, idx_Vout, idx_dVout, ...
                                         idx_Vin, idx_dVin, idx_Iload)
% EKF_PREDICT  Fully kinematic prediction — zero physical parameters.
%
%  All signals follow a constant-derivative (CV) model:
%    signal(k+1)     = signal(k) + Ts · deriv(k)
%    deriv(k+1)      = deriv(k)          ← random walk, driven by Q
%
%  Iload is an independent random walk:
%    Iload(k+1) = Iload(k)               ← NOT the node equation
%
%  The node equation  Σq_i·IL_i - C·dVout - Iload = 0  enters only
%  as a pseudo-measurement in ekf_update via the H matrix.

    n_states = 2*n_phases + 5;

    IL    = x(idx_IL);
    dIL   = x(idx_dIL);
    Vout  = x(idx_Vout);
    dVout = x(idx_dVout);
    Vin   = x(idx_Vin);
    dVin  = x(idx_dVin);
    Iload = x(idx_Iload);

    % ── State propagation f(x) ────────────────────────────────────────────
    x_pred = zeros(n_states, 1);
    x_pred(idx_IL)    = IL    + Ts * dIL;
    x_pred(idx_dIL)   = dIL;
    x_pred(idx_Vout)  = Vout  + Ts * dVout;
    x_pred(idx_dVout) = dVout;
    x_pred(idx_Vin)   = Vin   + Ts * dVin;
    x_pred(idx_dVin)  = dVin;
    x_pred(idx_Iload) = Iload;   % random walk

    % ── Jacobian F = ∂f/∂x ───────────────────────────────────────────────
    %
    %  F is purely kinematic — no dependence on the current state x,
    %  so this is exact (not an approximation):
    %
    %         IL_i    dIL_i   Vout   dVout   Vin   dVin   Iload
    %  IL_i  [  I      Ts·I    0      0       0     0       0  ]
    %  dIL_i [  0       I      0      0       0     0       0  ]
    %  Vout  [  0       0      1      Ts      0     0       0  ]
    %  dVout [  0       0      0      1       0     0       0  ]
    %  Vin   [  0       0      0      0       1     Ts      0  ]
    %  dVin  [  0       0      0      0       0     1       0  ]
    %  Iload [  0       0      0      0       0     0       1  ]  ← pure RW

    F = eye(n_states);
    for i = 1:n_phases
        F(idx_IL(i),  idx_dIL(i)) = Ts;
    end
    F(idx_Vout, idx_dVout) = Ts;
    F(idx_Vin,  idx_dVin)  = Ts;
    % Iload row stays identity (random walk, no cross-terms)

    % ── Covariance prediction ─────────────────────────────────────────────
    P_pred = F * P * F' + Q;
end


function [x_upd, P_upd] = ekf_update(x_pred, P_pred, z, H, R)
% EKF_UPDATE  Standard linear measurement update  (z = H·x is linear).
%
%  z includes both real measurements AND the pseudo-measurement row,
%  so Iload is softly pulled toward the node equation with weight
%  determined by sigma_Iload (the last diagonal of R).
%
%  Inputs
%    x_pred  Predicted state             [n_states x 1]
%    P_pred  Predicted covariance        [n_states x n_states]
%    z       Augmented measurement       [n_meas   x 1]  (last entry = 0)
%    H       Augmented measurement mat.  [n_meas   x n_states]
%    R       Augmented noise cov.        [n_meas   x n_meas]
%
%  Outputs
%    x_upd   Updated state
%    P_upd   Updated covariance (Joseph form)

    y = z - H * x_pred;          % innovation (node row: 0 - H_node·x_pred)
    S = H * P_pred * H' + R;     % innovation covariance
    K = P_pred * H' / S;         % Kalman gain

    x_upd = x_pred + K * y;

    % Joseph form for numerical stability
    I   = eye(size(P_pred));
    IKH = I - K * H;
    P_upd = IKH * P_pred * IKH' + K * R * K';
end