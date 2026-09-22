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
C_filt = [c_val; c_der];

n_states = 2*(n_phases + 2);

y_last = zeros(w, n_phases + 2);

%% Index
idx_IL    = 1:n_phases;
idx_Vout  = n_phases + 1;
idx_Vin   = n_phases + 2;
idx_IL_dot = idx_IL + n_meas;
idx_Vout_dot = idx_Vout + n_meas;
idx_Vin_dot = idx_Vin + n_meas;

%% Storage
% Simulation length from data
N = max(size(IL_m));
t = (0:N-1)*Ts;

x_hat_store = zeros(n_states, N);

%% Main Loop
for k = 1:N
    %% New measurements
    y_last = [y_last(2:end, :); IL_m(k, :) Vout_m(k) Vin_m(k)];

    y_hat = (C_filt*y_last)';
    x_hat = y_hat(:);

    %% Store
    x_hat_store(:, k) = x_hat;
end

%% Figure 1 – Phase inductor currents
figure('Name', 'Phase Currents');
for i = 1:n_phases
    subplot(n_phases, 1, i); hold on; grid on;
    plot(t, IL_m(:,i), 'LineWidth', 2, 'DisplayName', 'Measured');
    plot(t, x_hat_store(idx_IL(i),:), 'LineWidth', 2, 'DisplayName', 'Estimate');
    ylabel(sprintf('I_{L%d} (A)', i));
    legend('Location','best');
    if i == 1; title('Inductor Currents'); end
end
xlabel('Time (s)');

%% Figure 2 – Output voltage
figure('Name', 'Output Voltage'); hold on; grid on;
plot(t, Vout_m, 'LineWidth', 2, 'DisplayName', 'Measured');
plot(t, x_hat_store(idx_Vout,:), 'LineWidth', 2, 'DisplayName', 'Estimate');
ylabel('V_{out} (V)'); xlabel('Time (s)');
title('Output Voltage'); legend('Location','best');

%% Figure 3 – Input voltage
figure('Name', 'Input Voltage'); hold on; grid on;
plot(t, Vin_m, 'LineWidth', 2, 'DisplayName', 'Measured');
plot(t, x_hat_store(idx_Vin,:), 'LineWidth', 2, 'DisplayName', 'Estimate');
ylabel('V_{in} (V)'); xlabel('Time (s)');
title('Input Voltage'); legend('Location','best');

%% Figure 4 – Phase inductor currents derivatives
figure('Name', 'Phase Currents');
for i = 1:n_phases
    subplot(n_phases, 1, i); hold on; grid on;
    plot(t, x_hat_store(idx_IL_dot(i),:), 'LineWidth', 2, 'DisplayName', 'Estimate');
    ylabel(sprintf('I_{L%d} (A)', i));
    legend('Location','best');
    if i == 1; title('Inductor Currents Derivative'); end
end
xlabel('Time (s)');

%% Figure 5 – Output voltage derivative
figure('Name', 'Output Voltage'); hold on; grid on;
plot(t, x_hat_store(idx_Vout_dot,:), 'LineWidth', 2, 'DisplayName', 'Estimate');
ylabel('V_{out} (V)'); xlabel('Time (s)');
title('Output Voltage Derivative'); legend('Location','best');

%% Figure 6 – Input voltage derivative
figure('Name', 'Input Voltage'); hold on; grid on;
plot(t, x_hat_store(idx_Vin_dot,:), 'LineWidth', 2, 'DisplayName', 'Estimate');
ylabel('V_{in} (V)'); xlabel('Time (s)');
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