clear; clc; close all;

%% Load data
load('boost_data.mat')

n_phases = min(size(IL_m));
IL_m = IL_m';
n_meas = n_phases + 2;

u = 0.5*ones(n_phases, 1);

%% System parameters (initial estimates)
L_nom = 80e-3;
C_nom = 12e-3;
L_est    = L_nom * ones(n_phases, 1);
RL_est   = 10e-3 * ones(n_phases, 1);
VD_est   = 0.8   * ones(n_phases, 1);
C_est    = C_nom;
I_load_est = 0;       % <-- estimated as a single signal, Stage 1

G_est    = 0;         % \
Idc_est  = 0;         %  } Stage 2: load decomposition
CP_est   = 0;         % /

%% ── Stage 1 adaptation gains (physical equations) ────────────────────────
% Inductor equation residual drives: L, RL, VD
% Capacitor equation residual drives: C, I_load_est
gamma_L   = 1e-4  * ones(n_phases, 1);   % [H^-1 s^-1]
gamma_RL  = 5e-2  * ones(n_phases, 1);   % [Ohm^-1 s^-1]
gamma_VD  = 5e-1  * ones(n_phases, 1);   % [V^-1 s^-1]
gamma_C   = 1e-4;                         % [F^-1 s^-1]
gamma_Iload  = 1e3;    % adapts the scalar load current estimate

%% ── Stage 2 adaptation gains (load model decomposition) ─────────────────
% Residual: r_load = I_load_est - (G*Vout + Idc + CP/Vout)
gamma_G   = 1e-1;
gamma_Idc = 1e1;
gamma_CP  = 1e-1;

%% Bounds
tol_L = 0.1;
L_min = L_nom*(1-tol_L);   L_max = L_nom*(1+tol_L);
RL_min = 0.001;             RL_max = 0.1;
VD_min = 0.7;               VD_max = 1.5;
tol_C = 0.1;
C_min = C_nom*(1-tol_C);   C_max = C_nom*(1+tol_C);
P_max = 15000;              Vout_ref = 96;
Iload_min = 0;              Iload_max = P_max / 1;   % loose bound on total load current
G_min  = 0;                 G_max   = P_max/(Vout_ref^2);
Idc_min = 0;                Idc_max = P_max/Vout_ref;
CP_min  = 0;                CP_max  = P_max;

%% Savitzky-Golay filter
m = 12;
w = 2*m + 1;
d = 1;
[c_val, c_der] = savgol_coeffs(m, d, m, Ts);
C_filt  = [c_val; c_der];
n_states = 2*(n_phases + 2);
y_last  = zeros(w, n_phases + 2);

%% Signal indices
idx_IL       = 1:n_phases;
idx_Vout     = n_phases + 1;
idx_Vin      = n_phases + 2;
idx_IL_dot   = idx_IL   + n_meas;
idx_Vout_dot = idx_Vout + n_meas;
idx_Vin_dot  = idx_Vin  + n_meas;

%% Storage
N = max(size(IL_m));
t = (0:N-1)*Ts;

x_hat_store          = zeros(n_states, N);
IL_dot_model_store   = zeros(n_phases, N);
Vout_dot_model_store = zeros(1, N);
r_L_store            = zeros(n_phases, N);
r_C_store            = zeros(1, N);
r_load_store         = zeros(1, N);

% Stage 1
L_store      = zeros(n_phases, N);
RL_store     = zeros(n_phases, N);
VD_store     = zeros(n_phases, N);
C_store      = zeros(1, N);
Iload_store  = zeros(1, N);

% Stage 2
G_store      = zeros(1, N);
Idc_store    = zeros(1, N);
CP_store     = zeros(1, N);

% Power
Pin_store    = zeros(1, N);
Pout_store   = zeros(1, N);
PR_store     = zeros(1, N);
PD_store     = zeros(1, N);
PL_store     = zeros(1, N);
PC_store     = zeros(1, N);

%% ── Main Loop ─────────────────────────────────────────────────────────────
for k = 1:N
    %% S-G filtering
    y_last = [y_last(2:end, :); IL_m(k,:) Vout_m(k) Vin_m(k)];
    y_hat  = (C_filt * y_last)';
    x_hat  = y_hat(:);

    IL       = x_hat(idx_IL);
    Vout     = x_hat(idx_Vout);
    Vin      = x_hat(idx_Vin);
    IL_dot   = x_hat(idx_IL_dot);
    Vout_dot = x_hat(idx_Vout_dot);

    %% Model-predicted derivatives
    IL_dot_model   = (Vin - RL_est.*IL - (Vout + VD_est).*(1 - u)) ./ L_est;
    Vout_dot_model = (sum(IL.*(1 - u)) - I_load_est) / C_est;

    %% Power (pre-update)
    I_load_decomp  = G_est*Vout + Idc_est + CP_est/(Vout + 1e-3);
    Pin_store(k)   = Vin * sum(IL);
    Pout_store(k)  = Vout * I_load_est;
    PR_store(k)    = sum(RL_est .* IL.^2);
    PD_store(k)    = sum(VD_est .* IL .* (1 - u));
    PL_store(k)    = sum(L_est  .* IL .* IL_dot);
    PC_store(k)    = C_est * Vout * Vout_dot;

    %% ── STAGE 1: physical equation residuals ─────────────────────────────
    %
    % Inductor (per phase):
    %   r_L_i = [Vin - Vout*(1-u_i)] - [L_i*İL_i + RL_i*IL_i + VD_i*(1-u_i)]
    %
    % Capacitor:
    %   r_C   = [Σ IL_i*(1-u_i) - I_load_est] - C*V̇out
    %         = Σ IL_i*(1-u_i) - I_load_est - C*V̇out
    %
    r_L = (Vin - Vout.*(1 - u)) ...
        - (L_est.*IL_dot + RL_est.*IL + VD_est.*(1 - u));

    r_C = sum(IL.*(1 - u)) - I_load_est - C_est*Vout_dot;

    %% Stage 1 gradient updates
    %
    % ∂r_L_i/∂L_i  = -İL_i   →  ΔL_i  = +γ_L  · r_L_i · İL_i
    % ∂r_L_i/∂RL_i = -IL_i   →  ΔRL_i = +γ_RL · r_L_i · IL_i
    % ∂r_L_i/∂VD_i = -(1-u)  →  ΔVD_i = +γ_VD · r_L_i · (1-u_i)
    % ∂r_C/∂C      = -V̇out   →  ΔC    = +γ_C  · r_C   · V̇out
    % ∂r_C/∂I_load = -1      →  ΔI_load = +γ_Iload · r_C
    %
    L_est      = L_est      + Ts .* gamma_L      .* r_L .* IL_dot;
    RL_est     = RL_est     + Ts .* gamma_RL     .* r_L .* IL;
    VD_est     = VD_est     + Ts .* gamma_VD     .* r_L .* (1 - u);
    C_est      = C_est      + Ts  * gamma_C      *  r_C *  Vout_dot;
    I_load_est = I_load_est + Ts  * gamma_Iload  *  r_C;

    % Apply Stage 1 bounds
    L_est      = min(max(L_est,      L_min),        L_max);
    RL_est     = min(max(RL_est,     RL_min),        RL_max);
    VD_est     = min(max(VD_est,     VD_min),        VD_max);
    C_est      = min(max(C_est,      C_min),         C_max);
    I_load_est = min(max(I_load_est, Iload_min),     Iload_max);

    %% ── STAGE 2: load model decomposition ───────────────────────────────
    %
    %   I_load = G*Vout + Idc + CP/Vout
    %
    %   r_load = I_load_est - (G*Vout + Idc + CP/Vout)
    %
    % ∂r_load/∂G   = -Vout     →  ΔG   = +γ_G   · r_load · Vout
    % ∂r_load/∂Idc = -1        →  ΔIdc = +γ_Idc · r_load
    % ∂r_load/∂CP  = -1/Vout   →  ΔCP  = +γ_CP  · r_load / Vout
    %
    r_load = I_load_est - (G_est*Vout + Idc_est + CP_est/(Vout + 1e-3));

    G_est   = G_est   + Ts * gamma_G   * r_load * Vout;
    Idc_est = Idc_est + Ts * gamma_Idc * r_load;
    CP_est  = CP_est  + Ts * gamma_CP  * r_load / (Vout + 1e-3);

    % Apply Stage 2 bounds
    G_est   = min(max(G_est,   G_min),   G_max);
    Idc_est = min(max(Idc_est, Idc_min), Idc_max);
    CP_est  = min(max(CP_est,  CP_min),  CP_max);

    %% Store
    x_hat_store(:, k)        = x_hat;
    IL_dot_model_store(:, k) = IL_dot_model;
    Vout_dot_model_store(k)  = Vout_dot_model;
    r_L_store(:, k)          = r_L;
    r_C_store(k)             = r_C;
    r_load_store(k)          = r_load;

    L_store(:, k)    = L_est;
    RL_store(:, k)   = RL_est;
    VD_store(:, k)   = VD_est;
    C_store(k)       = C_est;
    Iload_store(k)   = I_load_est;
    G_store(k)       = G_est;
    Idc_store(k)     = Idc_est;
    CP_store(k)      = CP_est;
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

%% Figure 6 – MRAC residuals
figure('Name','MRAC Residuals');
subplot(3,1,1); hold on; grid on;
for i = 1:n_phases
    plot(t, r_L_store(i,:),'LineWidth',1.5,'DisplayName',sprintf('r_{L%d}',i));
end
yline(0,'k--'); ylabel('Residual (V)');
title('Stage 1 – Inductor Residuals  r_L'); legend('Location','best');

subplot(3,1,2); hold on; grid on;
plot(t, r_C_store,'LineWidth',1.5);
yline(0,'k--'); ylabel('Residual (A)');
title('Stage 1 – Capacitor Residual  r_C');

subplot(3,1,3); hold on; grid on;
plot(t, r_load_store,'LineWidth',1.5);
yline(0,'k--'); ylabel('Residual (A)');
title('Stage 2 – Load Decomposition Residual  r_{load}');
xlabel('Time (s)');

%% Figure 7 – Inductance L
figure('Name','Inductance');
for i = 1:n_phases
    subplot(n_phases,1,i); hold on; grid on;
    plot(t, L_store(i,:)*1e3,'LineWidth',2);
    yline(L_nom*1e3,'k--','Nominal');
    ylabel(sprintf('L_%d (mH)',i));
    if i==1; title('Inductance Estimates'); end
end
xlabel('Time (s)');

%% Figure 8 – Inductor resistance RL
figure('Name','Inductor Resistance');
for i = 1:n_phases
    subplot(n_phases,1,i); hold on; grid on;
    plot(t, RL_store(i,:)*1e3,'LineWidth',2);
    ylabel(sprintf('R_{L%d} (m\\Omega)',i));
    if i==1; title('Inductor Resistance Estimates'); end
end
xlabel('Time (s)');

%% Figure 9 – Diode voltage VD
figure('Name','Diode Voltage');
for i = 1:n_phases
    subplot(n_phases,1,i); hold on; grid on;
    plot(t, VD_store(i,:),'LineWidth',2);
    ylabel(sprintf('V_{D%d} (V)',i));
    if i==1; title('Diode Voltage Estimates'); end
end
xlabel('Time (s)');

%% Figure 10 – Capacitance C
figure('Name','Capacitance'); hold on; grid on;
plot(t, C_store*1e3,'LineWidth',2);
yline(C_nom*1e3,'k--','Nominal');
ylabel('C (mF)'); xlabel('Time (s)');
title('Capacitance Estimate');

%% Figure 11 – Load current estimate vs decomposition
figure('Name','Load Current');
subplot(2,1,1); hold on; grid on;
plot(t, Iload_store,'LineWidth',2,'DisplayName','I_{load} (Stage 1)');
plot(t, G_store.*x_hat_store(idx_Vout,:) + Idc_store + CP_store./(x_hat_store(idx_Vout,:)+1e-3), ...
    '--','LineWidth',1.5,'DisplayName','G·V_{out}+I_{dc}+CP/V_{out} (Stage 2)');
ylabel('I_{load} (A)'); legend('Location','best');
title('Load Current: Stage 1 Estimate vs Stage 2 Reconstruction');

subplot(2,1,2); hold on; grid on;
plot(t, r_load_store,'LineWidth',1.5);
yline(0,'k--');
ylabel('r_{load} (A)');
title('Stage 2 Decomposition Residual');
xlabel('Time (s)');

%% Figure 12 – Load parameters (Stage 2)
figure('Name','Load Parameters');
subplot(3,1,1); hold on; grid on;
plot(t, G_store,  'LineWidth',2);
ylabel('G_{load} (S)'); title('Load Parameter Estimates (Stage 2)');

subplot(3,1,2); hold on; grid on;
plot(t, Idc_store,'LineWidth',2);
ylabel('I_{dc,load} (A)');

subplot(3,1,3); hold on; grid on;
plot(t, CP_store, 'LineWidth',2);
ylabel('P_{cp,load} (W)'); xlabel('Time (s)');

%% Figure 13 – Power Conservation
figure('Name','Power Conservation');
subplot(2,1,1); hold on; grid on;
plot(t, Pin_store,'k','LineWidth',2,'DisplayName','P_{in}');
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
    area(t, data_neg,'FaceColor',[0.5 0.5 0.5],'FaceAlpha',0.3, ...
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