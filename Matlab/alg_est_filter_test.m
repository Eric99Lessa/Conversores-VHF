clear; clc; close all;

%% Load data
load('boost_data.mat')

n_phases = min(size(IL_m));

%% Make measurement data row-consistent
IL_m = IL_m';
n_meas = n_phases + 2;

%% Window size
m = 15;
w = 2*m + 1;
d = 1; % first order

[c_val, c_der] = alg_est_coeffs(m, 2*m*Ts, Ts);
C = [c_val; c_der];

n_states = 2*(n_phases + 2);

y_last = zeros(w, n_phases + 2);

%% Storage
% Simulation length from data
N = max(size(IL_m));
t = (0:N-1)*Ts;

x_hat_store = zeros(n_states, N);

%% Main Loop
for k = 1:N
    %% New measurements
    y_last = [y_last(2:end, :); IL_m(k, :) Vout_m(k) Vin_m(k)];

    y_hat = (C*y_last)';
    x_hat = y_hat(:);

    %% Store
    x_hat_store(:, k) = x_hat;
end

idx_IL    = 1:n_phases;
idx_Vout  = n_phases + 1;
idx_Vin   = n_phases + 2;
idx_IL_dot = idx_IL + n_meas;
idx_Vout_dot = idx_Vout + n_meas;
idx_Vin_dot = idx_Vin + n_meas;

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