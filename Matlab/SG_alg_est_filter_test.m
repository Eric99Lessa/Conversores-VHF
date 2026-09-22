clear; clc; close all;

%% Load data
load('boost_data.mat')

n_phases = min(size(IL_m));

%% Make measurement data row-consistent
IL_m = IL_m';
n_meas = n_phases + 2;

%% Window size
m = 12;
w = 2*m + 1;
d = 1; % first order

[c_val, c_der] = savgol_coeffs(m, d, m, Ts);
C_SG = [c_val; c_der];

[c_val, c_der] = alg_est_coeffs(m, 2*m*Ts, Ts);
C_alg_est = [c_val; c_der];

n_states = 2*(n_phases + 2);

y_last = zeros(w, n_phases + 2);

%% Storage
% Simulation length from data
N = max(size(IL_m));
t = (0:N-1)*Ts;

x_hat_store_alg_est = zeros(n_states, N);
x_hat_store_SG = zeros(n_states, N);

%% Main Loop
for k = 1:N
    %% New measurements
    y_last = [y_last(2:end, :); IL_m(k, :) Vout_m(k) Vin_m(k)];

    y_hat_alg_est = (C_alg_est*y_last)';
    x_hat_alg_est = y_hat_alg_est(:);

    y_hat_SG = (C_SG*y_last)';
    x_hat_SG = y_hat_SG(:);

    %% Store
    x_hat_store_alg_est(:, k) = x_hat_alg_est;
    x_hat_store_SG(:, k) = x_hat_SG;
end

idx_IL    = 1:n_phases;
idx_Vout  = n_phases + 1;
idx_Vin   = n_phases + 2;
idx_IL_dot = idx_IL + n_meas;
idx_Vout_dot = idx_Vout + n_meas;
idx_Vin_dot = idx_Vin + n_meas;

%% RMSE Calculation
% Interpolate true values to match filter time grid (t)
IL_true  = interp1(tc, IL,    t, 'linear', 'extrap');   % [N x n_phases]
Vin_true = interp1(tc, Vin,   t, 'linear', 'extrap')';   % [N x 1]
Vout_true= interp1(tc, Vout,  t, 'linear', 'extrap')';   % [N x 1]

rmse = @(est, true) sqrt(mean((est - true).^2));

fprintf('=== RMSE Comparison ===\n');
for i = 1:n_phases
    r1 = rmse(x_hat_store_SG (idx_IL(i),:)', IL_true(:,i));
    r2 = rmse(x_hat_store_alg_est(idx_IL(i),:)', IL_true(:,i));
    fprintf('IL%d  | Savitzky-Golay: %.4f  | Algebraic Estimator: %.4f\n', i, r1, r2);
end
r1 = rmse(x_hat_store_SG (idx_Vout,:)', Vout_true);
r2 = rmse(x_hat_store_alg_est(idx_Vout,:)', Vout_true);
fprintf('Vout | Savitzky-Golay: %.4f  | Algebraic Estimator: %.4f\n', r1, r2);
r1 = rmse(x_hat_store_SG (idx_Vin,:)', Vin_true);
r2 = rmse(x_hat_store_alg_est(idx_Vin,:)', Vin_true);
fprintf('Vin  | Savitzky-Golay: %.4f  | Algebraic Estimator: %.4f\n', r1, r2);

%% Figure 1 – Phase inductor currents + error
figure('Name', 'Phase Currents');
for i = 1:n_phases
    % Signal comparison
    subplot(n_phases, 2, 2*i-1); hold on; grid on;
    plot(tc, IL(:,i), 'LineWidth', 1.5, 'DisplayName', 'True');
    plot(t,  x_hat_store_SG (idx_IL(i),:), 'LineWidth', 1.5, 'DisplayName', 'Savitzky-Golay');
    plot(t,  x_hat_store_alg_est(idx_IL(i),:), '--', 'LineWidth', 1.5, 'DisplayName', 'Algebraic Estimator');
    ylabel(sprintf('I_{L%d} (A)', i)); legend('Location','best'); grid on;
    if i == 1; title('Inductor Currents'); end

    % Error
    subplot(n_phases, 2, 2*i); hold on; grid on;
    plot(t, x_hat_store_SG (idx_IL(i),:) - IL_true(:,i)', 'LineWidth', 1.5, 'DisplayName', 'Savitzky-Golay error');
    plot(t, x_hat_store_alg_est(idx_IL(i),:) - IL_true(:,i)', 'LineWidth', 1.5, 'DisplayName', 'Algebraic Estimator error');
    yline(0, 'k--'); ylabel(sprintf('\\Delta I_{L%d} (A)', i)); legend('Location','best');
    if i == 1; title('Error (Estimate - True)'); end
end
xlabel('Time (s)');

%% Figure 2 – Output voltage + error
figure('Name', 'Output Voltage');
subplot(2,1,1); hold on; grid on;
plot(tc, Vout, 'LineWidth', 1.5, 'DisplayName', 'True');
plot(t,  x_hat_store_SG (idx_Vout,:), 'LineWidth', 1.5, 'DisplayName', 'Savitzky-Golay');
plot(t,  x_hat_store_alg_est(idx_Vout,:), 'LineWidth', 1.5, 'DisplayName', 'Algebraic Estimator');
ylabel('V_{out} (V)'); title('Output Voltage'); legend('Location','best');

subplot(2,1,2); hold on; grid on;
plot(t, x_hat_store_SG (idx_Vout,:) - Vout_true', 'LineWidth', 1.5, 'DisplayName', 'Savitzky-Golay error');
plot(t, x_hat_store_alg_est(idx_Vout,:) - Vout_true', 'LineWidth', 1.5, 'DisplayName', 'Algebraic Estimator error');
yline(0,'k--'); ylabel('\Delta V_{out} (V)'); xlabel('Time (s)'); legend('Location','best');

%% Figure 3 – Input voltage + error
figure('Name', 'Input Voltage');
subplot(2,1,1); hold on; grid on;
plot(tc, Vin, 'LineWidth', 1.5, 'DisplayName', 'True');
plot(t,  x_hat_store_SG (idx_Vin,:), 'LineWidth', 1.5, 'DisplayName', 'Savitzky-Golay');
plot(t,  x_hat_store_alg_est(idx_Vin,:), 'LineWidth', 1.5, 'DisplayName', 'Algebraic Estimator');
ylabel('V_{in} (V)'); title('Input Voltage'); legend('Location','best');

subplot(2,1,2); hold on; grid on;
plot(t, x_hat_store_SG (idx_Vin,:) - Vin_true', 'LineWidth', 1.5, 'DisplayName', 'Savitzky-Golay error');
plot(t, x_hat_store_alg_est(idx_Vin,:) - Vin_true', 'LineWidth', 1.5, 'DisplayName', 'Algebraic Estimator error');
yline(0,'k--'); ylabel('\Delta V_{in} (V)'); xlabel('Time (s)'); legend('Location','best');

%% Figures 4-6 – Derivatives (unchanged, no true value available)
figure('Name', 'Inductor Current Derivatives');
for i = 1:n_phases
    subplot(n_phases, 1, i); hold on; grid on;
    plot(t, x_hat_store_SG (idx_IL_dot(i),:), 'LineWidth', 1.5, 'DisplayName', 'Savitzky-Golay');
    plot(t, x_hat_store_alg_est(idx_IL_dot(i),:), 'LineWidth', 1.5, 'DisplayName', 'Algebraic Estimator');
    ylabel(sprintf('dI_{L%d}/dt', i)); legend('Location','best');
    if i == 1; title('Inductor Current Derivatives'); end
end
xlabel('Time (s)');

figure('Name', 'Output Voltage Derivative'); hold on; grid on;
plot(t, x_hat_store_SG (idx_Vout_dot,:), 'LineWidth', 1.5, 'DisplayName', 'Savitzky-Golay');
plot(t, x_hat_store_alg_est(idx_Vout_dot,:), 'LineWidth', 1.5, 'DisplayName', 'Algebraic Estimator');
ylabel('dV_{out}/dt'); xlabel('Time (s)');
title('Output Voltage Derivative'); legend('Location','best');

figure('Name', 'Input Voltage Derivative'); hold on; grid on;
plot(t, x_hat_store_SG (idx_Vin_dot,:), 'LineWidth', 1.5, 'DisplayName', 'Savitzky-Golay');
plot(t, x_hat_store_alg_est(idx_Vin_dot,:), 'LineWidth', 1.5, 'DisplayName', 'Algebraic Estimator');
ylabel('dV_{in}/dt'); xlabel('Time (s)');
title('Input Voltage Derivative'); legend('Location','best');

function [c_val, c_der] = savgol_coeffs(m, d, x_eval, dx)
%SAVGOL_END_COEFFS Savitzky-Golay coefficients at end of window.
%
% [c_val, c_der] = savgol_end_coeffs(m, d, dx)
%
% Window: x = -m:m
% Evaluation point: x = m
%
% c_val : coefficients for value estimate
% c_der : coefficients for first derivative estimate

    if nargin < 3
        dx = 1;
    end

    x = (-m:m).';

    if d >= length(x)
        error('Polynomial degree d must be smaller than window length.');
    end

    if d < 1
        error('Polynomial degree d must be at least 1 for first derivative.');
    end

    % Design matrix
    A = zeros(length(x), d+1);
    for j = 0:d
        A(:, j+1) = x.^j;
    end

    % Value evaluation vector
    v_val = zeros(d+1, 1);
    for j = 0:d
        v_val(j+1) = x_eval^j;
    end

    % First derivative evaluation vector
    v_der = zeros(d+1, 1);
    for j = 1:d
        v_der(j+1) = j * x_eval^(j-1) / dx;
    end

    % Least-squares projection
    B = (A.' * A) \ A.';

    c_val = v_val.' * B;
    c_der = v_der.' * B;
end

function [c_val, c_der] = alg_est_coeffs(m, x_eval, dx)
    if nargin < 3
        dx = 1;
    end

    t_int = 2*m*dx;
    taus_int = 0:dx:t_int;
    w_int = [1 2*ones(1, 2*m - 1) 1];
    
    f_int = (2*t_int- 3.*taus_int);
    aux_int = w_int.*f_int;
    c_val = (2/(t_int^2))*(dx/2)*aux_int;
    
    f_int = (t_int - 2.*taus_int);
    aux_int = w_int.*f_int;
    c_der = (-6/(t_int^3))*(dx/2)*aux_int;
    
    c_val = c_val + c_der*x_eval;
end