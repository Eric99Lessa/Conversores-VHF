clear; clc; close all;

%% Load data
load('boost_data.mat')

n_phases = min(size(IL_m));
IL_m = IL_m';
n_meas = n_phases + 2;

%% System parameters (initial estimates)
L_nom = 80e-3;
C_nom = 12e-3;
L_est   = L_nom  * ones(n_phases, 1);
RL_est  = 10e-3  * ones(n_phases, 1);
VD_est  = 0.8    * ones(n_phases, 1);
C_est   = C_nom;
G_est   = 0;
Idc_est = 0;
CP_est  = 0;

n_theta = 3*n_phases + 1 + 3;
theta = [L_est; RL_est; VD_est; C_est; G_est; Idc_est; CP_est];

%% MRAC Adaptation gains
% Tune these: larger = faster adaptation but more noise sensitivity
gamma_L   = 0*1e-4  * ones(n_phases, 1);   % [H^-1 s^-1]
gamma_RL  = 5e-3  * ones(n_phases, 1);   % [Ohm^-1 s^-1]
gamma_VD  = 5e-1  * ones(n_phases, 1);   % [V^-1 s^-1]
gamma_C   = 0*1e-4;                         % [F^-1 s^-1]
gamma_G   = 1e-1;                         % [S^-1 s^-1]
gamma_Idc = 1e1;                         % [A^-1 s^-1]
gamma_CP  = 1e-1;                        % [W^-1 s^-1]

%% Bounds
tol_L = 0.1;
L_min = L_nom*(1-tol_L);   L_max = L_nom*(1+tol_L);
RL_min = 0.001;             RL_max = 0.1;
VD_min = 0.7;               VD_max = 1.5;
tol_C = 0.1;
C_min = C_nom*(1-tol_C);   C_max = C_nom*(1+tol_C);
G_load_min   = 0;           Idc_load_min = 0;   CP_load_min = 0;
P_max = 15000;              Vout_ref = 96;
G_load_max   = P_max/(Vout_ref^2);
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
C_filt  = [c_val; c_der];
n_states = 2*(n_phases + 2);
y_last  = zeros(w, n_phases + 2);

%% Indices — signals
idx_IL       = 1:n_phases;
idx_Vout     = n_phases + 1;
idx_Vin      = n_phases + 2;
idx_IL_dot   = idx_IL   + n_meas;
idx_Vout_dot = idx_Vout + n_meas;
idx_Vin_dot  = idx_Vin  + n_meas;

%% Indices — parameters in theta
idx_t_L   = 1:n_phases;
idx_t_RL  = n_phases+1   : 2*n_phases;
idx_t_VD  = 2*n_phases+1 : 3*n_phases;
idx_t_C   = 3*n_phases+1;
idx_t_G   = 3*n_phases+2;
idx_t_Idc = 3*n_phases+3;
idx_t_CP  = 3*n_phases+4;

%% Storage
N = max(size(IL_m));
t = (0:N-1)*Ts;

x_hat_store          = zeros(n_states, N);
theta_store          = zeros(n_theta,  N);
IL_dot_model_store   = zeros(n_phases, N);
Vout_dot_model_store = zeros(1, N);
r_L_store            = zeros(n_phases, N);   % inductor residuals
r_C_store            = zeros(1, N);           % capacitor residual
Pin_store            = zeros(1, N);
Pout_store           = zeros(1, N);
PR_store             = zeros(1, N);
PD_store             = zeros(1, N);
PL_store             = zeros(1, N);
PC_store             = zeros(1, N);

%% ── Main Loop ─────────────────────────────────────────────────────────────
for k = 1:N

    %% S-G filtering
    y_last = [y_last(2:end, :); IL_m(k, :) Vout_m(k) Vin_m(k)];
    y_hat  = (C_filt * y_last)';
    x_hat  = y_hat(:);

    IL       = x_hat(idx_IL);
    Vout     = x_hat(idx_Vout);
    Vin      = x_hat(idx_Vin);
    IL_dot   = x_hat(idx_IL_dot);
    Vout_dot = x_hat(idx_Vout_dot);

    %% Unpack current theta
    L_est   = theta(idx_t_L);
    RL_est  = theta(idx_t_RL);
    VD_est  = theta(idx_t_VD);
    C_est   = theta(idx_t_C);
    G_est   = theta(idx_t_G);
    Idc_est = theta(idx_t_Idc);
    CP_est  = theta(idx_t_CP);

    %% Model-predicted derivatives
    I_load_est     = G_est*Vout + Idc_est + CP_est/(Vout + 1e-3);
    IL_dot_model   = (Vin - RL_est.*IL - (Vout + VD_est).*(1 - u)) ./ L_est;
    Vout_dot_model = (sum(IL.*(1 - u)) - I_load_est) / C_est;

    %% Power (pre-update estimates)
    Pin_store(k)  = Vin * sum(IL);
    Pout_store(k) = Vout * I_load_est;
    PR_store(k)   = sum(RL_est .* IL.^2);
    PD_store(k)   = sum(VD_est .* IL .* (1 - u));
    PL_store(k)   = sum(L_est  .* IL .* IL_dot);
    PC_store(k)   = C_est * Vout * Vout_dot;

    %% ── MRAC equation residuals ───────────────────────────────────────────
    %
    % Inductor (per phase i):
    %   L_i * İL_i + RL_i * IL_i + VD_i*(1-u_i) = Vin - Vout*(1-u_i)
    %   r_L_i  = [Vin - Vout*(1-u_i)] - [L_i*İL_i + RL_i*IL_i + VD_i*(1-u_i)]
    %
    % Capacitor:
    %   C * V̇out + G*Vout + Idc + CP/Vout = Σ IL_i*(1-u_i)
    %   r_C = Σ IL_i*(1-u_i) - [C*V̇out + G*Vout + Idc + CP/Vout]
    %
    r_L = (Vin - Vout.*(1 - u)) ...
        - (L_est.*IL_dot + RL_est.*IL + VD_est.*(1 - u));

    r_C = sum(IL.*(1 - u)) ...
        - (C_est*Vout_dot + G_est*Vout + Idc_est + CP_est/(Vout + 1e-3));

    %% ── Gradient adaptation laws  (Δθ = Ts·γ·r·∂r/∂θ) ───────────────────
    %
    % ∂r_L_i/∂L_i   = -İL_i   →  ΔL_i   = +Ts·γ_L  · r_L_i · İL_i
    % ∂r_L_i/∂RL_i  = -IL_i   →  ΔRL_i  = +Ts·γ_RL · r_L_i · IL_i
    % ∂r_L_i/∂VD_i  = -(1-u)  →  ΔVD_i  = +Ts·γ_VD · r_L_i · (1-u_i)
    % ∂r_C/∂C       = -V̇out   →  ΔC     = +Ts·γ_C  · r_C   · V̇out
    % ∂r_C/∂G       = -Vout   →  ΔG     = +Ts·γ_G  · r_C   · Vout
    % ∂r_C/∂Idc     = -1      →  ΔIdc   = +Ts·γ_Idc· r_C
    % ∂r_C/∂CP      = -1/Vout →  ΔCP    = +Ts·γ_CP · r_C   · (1/Vout)
    %
    L_est   = L_est   + Ts .* gamma_L   .* r_L .* IL_dot;
    RL_est  = RL_est  + Ts .* gamma_RL  .* r_L .* IL;
    VD_est  = VD_est  + Ts .* gamma_VD  .* r_L .* (1 - u);
    C_est   = C_est   + Ts  * gamma_C   * r_C  * Vout_dot;
    G_est   = G_est   + Ts  * gamma_G   * r_C  * Vout;
    Idc_est = Idc_est + Ts  * gamma_Idc * r_C;
    CP_est  = CP_est  + Ts  * gamma_CP  * r_C  / (Vout + 1e-3);

    theta = [L_est; RL_est; VD_est; C_est; G_est; Idc_est; CP_est];

    %% Apply bounds
    theta = min(max(theta, theta_min), theta_max);

    %% Store
    x_hat_store(:, k)        = x_hat;
    theta_store(:, k)        = theta;
    IL_dot_model_store(:, k) = IL_dot_model;
    Vout_dot_model_store(k)  = Vout_dot_model;
    r_L_store(:, k)          = r_L;
    r_C_store(k)             = r_C;
end

%% ── PLOTS ─────────────────────────────────────────────────────────────────

%% Figure 1 – Phase inductor currents
figure('Name','Phase Currents');
for i = 1:n_phases
    subplot(n_phases,1,i); hold on; grid on;
    plot(t, IL_m(:,i),                'LineWidth',2,'DisplayName','Measured');
    plot(t, x_hat_store(idx_IL(i),:), 'LineWidth',2,'DisplayName','Filtered');
    ylabel(sprintf('I_{L%d} (A)',i)); legend('Location','best');
    if i==1; title('Inductor Currents'); end
end
xlabel('Time (s)');

%% Figure 2 – Output voltage
figure('Name','Output Voltage'); hold on; grid on;
plot(t, Vout_m,                  'LineWidth',2,'DisplayName','Measured');
plot(t, x_hat_store(idx_Vout,:), 'LineWidth',2,'DisplayName','Filtered');
ylabel('V_{out} (V)'); xlabel('Time (s)');
title('Output Voltage'); legend('Location','best');

%% Figure 3 – Input voltage
figure('Name','Input Voltage'); hold on; grid on;
plot(t, Vin_m,                  'LineWidth',2,'DisplayName','Measured');
plot(t, x_hat_store(idx_Vin,:), 'LineWidth',2,'DisplayName','Filtered');
ylabel('V_{in} (V)'); xlabel('Time (s)');
title('Input Voltage'); legend('Location','best');

%% Figure 4 – Inductor current derivatives (S-G vs model)
figure('Name','Inductor Current Derivatives');
for i = 1:n_phases
    subplot(n_phases,1,i); hold on; grid on;
    plot(t, x_hat_store(idx_IL_dot(i),:), 'LineWidth',2,'DisplayName','S-G Estimate');
    plot(t, IL_dot_model_store(i,:),  '--','LineWidth',2,'DisplayName','Model');
    ylabel(sprintf('dI_{L%d}/dt (A/s)',i)); legend('Location','best');
    if i==1; title('Inductor Current Derivatives'); end
end
xlabel('Time (s)');

%% Figure 5 – Output voltage derivative (S-G vs model)
figure('Name','Output Voltage Derivative'); hold on; grid on;
plot(t, x_hat_store(idx_Vout_dot,:), 'LineWidth',2,'DisplayName','S-G Estimate');
plot(t, Vout_dot_model_store,    '--','LineWidth',2,'DisplayName','Model');
ylabel('dV_{out}/dt (V/s)'); xlabel('Time (s)');
title('Output Voltage Derivative'); legend('Location','best');

%% Figure 6 – MRAC residuals (adaptation driving signal)
figure('Name','MRAC Residuals');
subplot(2,1,1); hold on; grid on;
for i = 1:n_phases
    plot(t, r_L_store(i,:), 'LineWidth',1.5,'DisplayName',sprintf('r_{L%d}',i));
end
yline(0,'k--'); ylabel('Residual (V)'); legend('Location','best');
title('Inductor Equation Residuals  r_L');

subplot(2,1,2); hold on; grid on;
plot(t, r_C_store, 'LineWidth',1.5);
yline(0,'k--'); ylabel('Residual (A)');
title('Capacitor Equation Residual  r_C');
xlabel('Time (s)');

%% Figure 7 – Inductance L
figure('Name','Inductance');
for i = 1:n_phases
    subplot(n_phases,1,i); hold on; grid on;
    plot(t, theta_store(idx_t_L(i),:)*1e3, 'LineWidth',2);
    yline(L_nom*1e3,'k--','Nominal');
    ylabel(sprintf('L_%d (mH)',i));
    if i==1; title('Inductance Estimates'); end
end
xlabel('Time (s)');

%% Figure 8 – Inductor resistance RL
figure('Name','Inductor Resistance');
for i = 1:n_phases
    subplot(n_phases,1,i); hold on; grid on;
    plot(t, theta_store(idx_t_RL(i),:)*1e3, 'LineWidth',2);
    ylabel(sprintf('R_{L%d} (m\\Omega)',i));
    if i==1; title('Inductor Resistance Estimates'); end
end
xlabel('Time (s)');

%% Figure 9 – Diode voltage VD
figure('Name','Diode Voltage');
for i = 1:n_phases
    subplot(n_phases,1,i); hold on; grid on;
    plot(t, theta_store(idx_t_VD(i),:), 'LineWidth',2);
    ylabel(sprintf('V_{D%d} (V)',i));
    if i==1; title('Diode Voltage Estimates'); end
end
xlabel('Time (s)');

%% Figure 10 – Capacitance C
figure('Name','Capacitance'); hold on; grid on;
plot(t, theta_store(idx_t_C,:)*1e3, 'LineWidth',2);
yline(C_nom*1e3,'k--','Nominal');
ylabel('C (mF)'); xlabel('Time (s)');
title('Capacitance Estimate');

%% Figure 11 – Load parameters
figure('Name','Load Parameters');
subplot(3,1,1); hold on; grid on;
plot(t, theta_store(idx_t_G,:),   'LineWidth',2);
ylabel('G_{load} (S)'); title('Load Parameter Estimates');
subplot(3,1,2); hold on; grid on;
plot(t, theta_store(idx_t_Idc,:), 'LineWidth',2);
ylabel('I_{dc,load} (A)');
subplot(3,1,3); hold on; grid on;
plot(t, theta_store(idx_t_CP,:),  'LineWidth',2);
ylabel('P_{cp,load} (W)'); xlabel('Time (s)');

%% Figure 12 – Power Conservation
figure('Name','Power Conservation');

subplot(2,1,1); hold on; grid on;
plot(t, Pin_store, 'k','LineWidth',2,'DisplayName','P_{in}');
plot(t, Pout_store + PR_store + PD_store + PL_store + PC_store, ...
    'r--','LineWidth',1.5,'DisplayName','P_{out}+losses (sum)');
ylabel('Power (W)'); legend('Location','best');
title('Input Power vs. Sum of Output and Losses');

subplot(2,1,2); hold on; grid on;
data_stack = [Pout_store; PR_store; PD_store; PL_store; PC_store]';
data_pos   = max(data_stack, 0);
data_neg   = min(data_stack, 0);
h_area = area(t, data_pos);
colors = lines(5);
labels = {'P_{out}','P_R (resistive)','P_D (diode)','P_L (inductor)','P_C (capacitor)'};
for i = 1:5
    h_area(i).FaceColor   = colors(i,:);
    h_area(i).FaceAlpha   = 0.7;
    h_area(i).DisplayName = labels{i};
end
if any(data_neg(:) < 0)
    area(t, data_neg, 'FaceColor',[0.5 0.5 0.5],'FaceAlpha',0.3, ...
        'DisplayName','Storage return (negative)');
end
plot(t, Pin_store,'k','LineWidth',2,'DisplayName','P_{in}');
ylabel('Power (W)'); xlabel('Time (s)');
title('Output Power and Losses (Stacked)'); legend('Location','best');

%% ── FUNCTIONS ─────────────────────────────────────────────────────────────

function [c_val, c_der] = savgol_coeffs(m, d, x_eval, dx)
    if nargin < 3; dx = 1; end
    x = (-m:m).';
    if d >= length(x); error('Polynomial degree d must be smaller than window length.'); end
    if d < 1;          error('Polynomial degree d must be at least 1 for first derivative.'); end
    A = zeros(length(x), d+1);
    for j = 0:d; A(:,j+1) = x.^j; end
    v_val = zeros(d+1,1);
    for j = 0:d; v_val(j+1) = x_eval^j; end
    v_der = zeros(d+1,1);
    for j = 1:d; v_der(j+1) = j * x_eval^(j-1) / dx; end
    B     = (A.'*A) \ A.';
    c_val = v_val.' * B;
    c_der = v_der.' * B;
end