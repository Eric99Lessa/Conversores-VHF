clear; clc; close all;

%% Load data
load('boost_data.mat')

n_phases = min(size(IL_m));

%% Make measurement data row-consistent
IL_m = IL_m';
n_meas = n_phases + 2;

%% System parameters
L_nom = 80e-3;
C_nom = 12e-3;
L = L_nom*ones(n_phases, 1);
RL = (10e-3)*ones(n_phases, 1);
VD = 0.8*ones(n_phases, 1);
C = C_nom;
G_load = 0;
Idc_load = 0;
CP_load = 0;

n_theta = 3*n_phases + 1 + 3;
theta = [L; RL; VD; C; G_load; Idc_load; CP_load];

%% --- RLS Method Selection ---
% 'scalar'  : standard P-form RLS with single forgetting factor
% 'vector'  : information-form RLS with per-parameter forgetting factors
rls_method = 'scalar';

%% Scalar lambda (P-form)
lambda_scalar = exp(-Ts/0.1);
invR = eye(n_theta);

%% Vector lambda (information form)
tau_slow = 10;
tau_fast = 1;
lambda_L   = exp(-Ts/tau_slow) * ones(n_phases, 1);
lambda_RL  = exp(-Ts/tau_slow) * ones(n_phases, 1);
lambda_VD  = exp(-Ts/tau_fast) * ones(n_phases, 1);
lambda_C   = exp(-Ts/tau_slow);
lambda_G   = exp(-Ts/tau_fast);
lambda_Idc = exp(-Ts/tau_fast);
lambda_CP  = exp(-Ts/tau_fast);
lambda_vec = [lambda_L; lambda_RL; lambda_VD; lambda_C; lambda_G; lambda_Idc; lambda_CP];

delta = 1e-4;
Omega = delta * eye(n_theta);
h = Omega * theta;

%% Bounds
tol_L = 0.1;
L_min = L_nom*(1 - tol_L);   L_max = L_nom*(1 + tol_L);
RL_min = 0.001;               RL_max = 0.1;
VD_min = 0.7;                 VD_max = 1.5;
tol_C = 0.1;
C_min = C_nom*(1 - tol_C);   C_max = C_nom*(1 + tol_C);
G_load_min = 0;               Idc_load_min = 0;   CP_load_min = 0;
P_max = 15000;                Vout_ref = 96;
G_load_max  = P_max/(Vout_ref^2);
Idc_load_max = P_max/Vout_ref;
CP_load_max  = P_max;
theta_min = [L_min*ones(n_phases,1); RL_min*ones(n_phases,1); VD_min*ones(n_phases,1);
             C_min; G_load_min; Idc_load_min; CP_load_min];
theta_max = [L_max*ones(n_phases,1); RL_max*ones(n_phases,1); VD_max*ones(n_phases,1);
             C_max; G_load_max; Idc_load_max; CP_load_max];

u = 0.5*ones(n_phases, 1);

%% Savitzky-Golay filter
m = 12;
w = 2*m + 1;
d = 1;
[c_val, c_der] = savgol_coeffs(m, d, m, Ts);
C_filt = [c_val; c_der];
n_states = 2*(n_phases + 2);
y_last = zeros(w, n_phases + 2);

%% Index
idx_IL       = 1:n_phases;
idx_Vout     = n_phases + 1;
idx_Vin      = n_phases + 2;
idx_IL_dot   = idx_IL   + n_meas;
idx_Vout_dot = idx_Vout + n_meas;
idx_Vin_dot  = idx_Vin  + n_meas;

%% Parameter indices in theta
idx_t_L    = 1:n_phases;
idx_t_RL   = n_phases+1 : 2*n_phases;
idx_t_VD   = 2*n_phases+1 : 3*n_phases;
idx_t_C    = 3*n_phases+1;
idx_t_G    = 3*n_phases+2;
idx_t_Idc  = 3*n_phases+3;
idx_t_CP   = 3*n_phases+4;

%% Storage
N = max(size(IL_m));
t = (0:N-1)*Ts;

x_hat_store   = zeros(n_states, N);
theta_store   = zeros(n_theta,  N);
IL_dot_model_store   = zeros(n_phases, N);
Vout_dot_model_store = zeros(1, N);
Pin_store    = zeros(1, N);
Pout_store   = zeros(1, N);
PR_store     = zeros(1, N);
PD_store     = zeros(1, N);
PL_store     = zeros(1, N);
PC_store     = zeros(1, N);

%% Main Loop
for k = 1:N
    %% New measurements
    y_last = [y_last(2:end, :); IL_m(k, :) Vout_m(k) Vin_m(k)];

    y_hat = (C_filt*y_last)';
    x_hat = y_hat(:);

    %% Get filtered values
    IL       = x_hat(idx_IL);
    Vout     = x_hat(idx_Vout);
    Vin      = x_hat(idx_Vin);
    IL_dot   = x_hat(idx_IL_dot);
    Vout_dot = x_hat(idx_Vout_dot);

    %% Extract current parameter estimates
    L_est    = theta(idx_t_L);
    RL_est   = theta(idx_t_RL);
    VD_est   = theta(idx_t_VD);
    C_est    = theta(idx_t_C);
    G_est    = theta(idx_t_G);
    Idc_est  = theta(idx_t_Idc);
    CP_est   = theta(idx_t_CP);

    %% Model-predicted derivatives (average boost converter model)
    I_load_est = G_est*Vout + Idc_est + CP_est/(Vout + 1e-3);

    % L*dIL/dt = Vin - RL*IL - (Vout + VD)*(1-u)
    IL_dot_model   = (Vin - RL_est.*IL - (Vout + VD_est).*(1 - u)) ./ L_est;
    % C*dVout/dt = sum(IL*(1-u)) - G*Vout - Idc - CP/Vout
    Vout_dot_model = (sum(IL.*(1 - u)) - I_load_est) / C_est;

    %% Power terms (using current parameter estimates)
    Pin_k  = Vin * sum(IL);
    Pout_k = Vout * I_load_est;
    PR_k   = sum(RL_est .* IL.^2);
    PD_k   = sum(VD_est .* IL .* (1 - u));
    PL_k   = sum(L_est  .* IL .* IL_dot);
    PC_k   = C_est * Vout * Vout_dot;

    Pin_store(k)  = Pin_k;
    Pout_store(k) = Pout_k;
    PR_store(k)   = PR_k;
    PD_store(k)   = PD_k;
    PL_store(k)   = PL_k;
    PC_store(k)   = PC_k;

    %% Regressor matrices
    phi_IL   = [diag(IL_dot); diag(IL); diag(1 - u); zeros(n_theta - 3*n_phases, n_phases)];
    phi_Vout = [zeros(3*n_phases, 1); Vout_dot; Vout; 1; 1/(Vout + 0.001)];
    phi_P    = [IL.*IL_dot; IL.^2; IL.*(1 - u); Vout*Vout_dot; Vout^2; Vout; 1];

    W   = diag([1 1 1 1 0.1]);
    phi = W * [phi_IL phi_Vout phi_P]';
    y   = W * [Vin - Vout*(1 - u); sum(IL.*(1 - u)); Vin*sum(IL)];

    %% RLS update
    switch rls_method
        case 'scalar'
            [theta, invR] = RLS_iter_scalar(theta, invR, phi, y, lambda_scalar, n_theta, size(phi,1));
        case 'vector'
            [theta, Omega, h] = RLS_iter_vector(Omega, h, phi, y, lambda_vec);
    end

    theta = min(max(theta, theta_min), theta_max);

    %% Store
    x_hat_store(:, k)          = x_hat;
    theta_store(:, k)          = theta;
    IL_dot_model_store(:, k)   = IL_dot_model;
    Vout_dot_model_store(k)    = Vout_dot_model;
end

%% ── PLOTS ─────────────────────────────────────────────────────────────────

%% Figure 1 – Phase inductor currents
figure('Name', 'Phase Currents');
for i = 1:n_phases
    subplot(n_phases, 1, i); hold on; grid on;
    plot(t, IL_m(:,i),              'LineWidth', 2, 'DisplayName', 'Measured');
    plot(t, x_hat_store(idx_IL(i),:),'LineWidth', 2, 'DisplayName', 'Filtered');
    ylabel(sprintf('I_{L%d} (A)', i)); legend('Location','best');
    if i == 1; title('Inductor Currents'); end
end
xlabel('Time (s)');

%% Figure 2 – Output voltage
figure('Name', 'Output Voltage'); hold on; grid on;
plot(t, Vout_m,                     'LineWidth', 2, 'DisplayName', 'Measured');
plot(t, x_hat_store(idx_Vout,:),    'LineWidth', 2, 'DisplayName', 'Filtered');
ylabel('V_{out} (V)'); xlabel('Time (s)');
title('Output Voltage'); legend('Location','best');

%% Figure 3 – Input voltage
figure('Name', 'Input Voltage'); hold on; grid on;
plot(t, Vin_m,                      'LineWidth', 2, 'DisplayName', 'Measured');
plot(t, x_hat_store(idx_Vin,:),     'LineWidth', 2, 'DisplayName', 'Filtered');
ylabel('V_{in} (V)'); xlabel('Time (s)');
title('Input Voltage'); legend('Location','best');

%% Figure 4 – Inductor current derivatives (filtered vs model)
figure('Name', 'Inductor Current Derivatives');
for i = 1:n_phases
    subplot(n_phases, 1, i); hold on; grid on;
    plot(t, x_hat_store(idx_IL_dot(i),:), 'LineWidth', 2, 'DisplayName', 'S-G Estimate');
    plot(t, IL_dot_model_store(i,:),       'LineWidth', 2, 'LineStyle', '--', 'DisplayName', 'Model');
    ylabel(sprintf('dI_{L%d}/dt (A/s)', i)); legend('Location','best');
    if i == 1; title('Inductor Current Derivatives'); end
end
xlabel('Time (s)');

%% Figure 5 – Output voltage derivative (filtered vs model)
figure('Name', 'Output Voltage Derivative'); hold on; grid on;
plot(t, x_hat_store(idx_Vout_dot,:), 'LineWidth', 2, 'DisplayName', 'S-G Estimate');
plot(t, Vout_dot_model_store,         'LineWidth', 2, 'LineStyle', '--', 'DisplayName', 'Model');
ylabel('dV_{out}/dt (V/s)'); xlabel('Time (s)');
title('Output Voltage Derivative'); legend('Location','best');

%% Figure 6 – Inductance L
figure('Name', 'Inductance');
for i = 1:n_phases
    subplot(n_phases, 1, i); hold on; grid on;
    plot(t, theta_store(idx_t_L(i),:)*1e3, 'LineWidth', 2);
    yline(L_nom*1e3, 'k--', 'Nominal');
    ylabel(sprintf('L_%d (mH)', i)); grid on;
    if i == 1; title('Inductance Estimates'); end
end
xlabel('Time (s)');

%% Figure 7 – Inductor resistance RL
figure('Name', 'Inductor Resistance');
for i = 1:n_phases
    subplot(n_phases, 1, i); hold on; grid on;
    plot(t, theta_store(idx_t_RL(i),:)*1e3, 'LineWidth', 2);
    ylabel(sprintf('R_{L%d} (m\\Omega)', i)); grid on;
    if i == 1; title('Inductor Resistance Estimates'); end
end
xlabel('Time (s)');

%% Figure 8 – Diode voltage VD
figure('Name', 'Diode Voltage');
for i = 1:n_phases
    subplot(n_phases, 1, i); hold on; grid on;
    plot(t, theta_store(idx_t_VD(i),:), 'LineWidth', 2);
    ylabel(sprintf('V_{D%d} (V)', i)); grid on;
    if i == 1; title('Diode Voltage Estimates'); end
end
xlabel('Time (s)');

%% Figure 9 – Capacitance C
figure('Name', 'Capacitance'); hold on; grid on;
plot(t, theta_store(idx_t_C,:)*1e3, 'LineWidth', 2);
yline(C_nom*1e3, 'k--', 'Nominal');
ylabel('C (mF)'); xlabel('Time (s)');
title('Capacitance Estimate');

%% Figure 10 – Load parameters
figure('Name', 'Load Parameters');

subplot(3,1,1); hold on; grid on;
plot(t, theta_store(idx_t_G,:), 'LineWidth', 2);
ylabel('G_{load} (S)');
title('Load Parameter Estimates');

subplot(3,1,2); hold on; grid on;
plot(t, theta_store(idx_t_Idc,:), 'LineWidth', 2);
ylabel('I_{dc,load} (A)');

subplot(3,1,3); hold on; grid on;
plot(t, theta_store(idx_t_CP,:), 'LineWidth', 2);
ylabel('P_{cp,load} (W)');
xlabel('Time (s)');

%% Figure 11 – Power Conservation Check
figure('Name', 'Power Conservation');

%% Top subplot: Input power
subplot(2,1,1); hold on; grid on;
plot(t, Pin_store, 'k', 'LineWidth', 2, 'DisplayName', 'P_{in}');
plot(t, Pout_store + PR_store + PD_store + PL_store + PC_store, ...
    'r--', 'LineWidth', 1.5, 'DisplayName', 'P_{out}+losses (sum)');
ylabel('Power (W)');
title('Input Power vs. Sum of Output and Losses');
legend('Location','best');

%% Bottom subplot: Stacked area of output + losses
subplot(2,1,2); hold on; grid on;

% Stack: Pout | PR | PD | PL | PC
% area() stacks positive values automatically
data_stack = [Pout_store; PR_store; PD_store; PL_store; PC_store]';
% Clamp negatives to zero for stacking (storage terms can be negative)
data_pos = max(data_stack, 0);
data_neg = min(data_stack, 0);

h_area = area(t, data_pos);
colors = lines(5);
labels = {'P_{out}','P_{R} (resistive)','P_{D} (diode)','P_{L} (inductor)','P_{C} (capacitor)'};
for i = 1:5
    h_area(i).FaceColor = colors(i,:);
    h_area(i).FaceAlpha = 0.7;
    h_area(i).DisplayName = labels{i};
end

% Overlay negative storage (PL/PC when energy is being stored back)
if any(data_neg(:) < 0)
    area(t, data_neg, 'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.3, ...
        'DisplayName', 'Storage return (negative)');
end

% Overlay Pin for reference
plot(t, Pin_store, 'k', 'LineWidth', 2, 'DisplayName', 'P_{in}');

ylabel('Power (W)');
xlabel('Time (s)');
title('Output Power and Losses (Stacked)');
legend('Location','best');

%% ── FUNCTIONS ─────────────────────────────────────────────────────────────

function [c_val, c_der] = savgol_coeffs(m, d, x_eval, dx)
    if nargin < 3; dx = 1; end
    x = (-m:m).';
    if d >= length(x); error('Polynomial degree d must be smaller than window length.'); end
    if d < 1; error('Polynomial degree d must be at least 1 for first derivative.'); end
    A = zeros(length(x), d+1);
    for j = 0:d; A(:, j+1) = x.^j; end
    v_val = zeros(d+1, 1);
    for j = 0:d; v_val(j+1) = x_eval^j; end
    v_der = zeros(d+1, 1);
    for j = 1:d; v_der(j+1) = j * x_eval^(j-1) / dx; end
    B = (A.' * A) \ A.';
    c_val = v_val.' * B;
    c_der = v_der.' * B;
end

% ── Standard P-form RLS (scalar forgetting factor) ────────────────────────
function [theta_i, invR_i] = RLS_iter_scalar(theta_prev, invR_prev, phi, y, lambda, n_theta, n_eq)
    S      = lambda * eye(n_eq) + phi * invR_prev * phi';
    Kk     = invR_prev * phi' / S;
    invR_i = (1/lambda) * (eye(n_theta) - Kk * phi) * invR_prev;
    invR_i = (invR_i + invR_i') / 2;          % enforce symmetry
    theta_i = theta_prev + Kk * (y - phi * theta_prev);
end

% ── Information-form RLS (per-parameter forgetting vector) ────────────────
function [theta_i, Omega_i, h_i] = RLS_iter_vector(Omega_prev, h_prev, phi, y, lambda_vec)
    Lambda_sq = sqrt(lambda_vec) * sqrt(lambda_vec)';  % n_theta x n_theta
    Omega_i   = Lambda_sq .* Omega_prev + phi' * phi;
    h_i       = lambda_vec .* h_prev    + phi' * y;
    theta_i   = Omega_i \ h_i;
end