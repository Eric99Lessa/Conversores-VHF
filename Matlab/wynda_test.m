clear; clc; close all;

%% Load data
load('boost_data.mat')

n_phases = min(size(IL_m));
IL_m = IL_m';

u = 0.5*ones(n_phases, 1);

%% Savitzky-Golay filter
m = 12;
w = 2*m + 1;
d = 1;
[c_val, c_der] = savgol_coeffs(m, d, m, Ts);
C_filt  = [c_val; c_der];
n_states = 2*(n_phases + 2);
Vin_last = zeros(w, 1);
Vout_last = zeros(w, 1);

%% ── WyNDA setup ───────────────────────────────────────────────────────────
%  States:  x = [IL_1,...,IL_n, Vout]'       (n_x × 1)
%  Params:  theta = [alpha_i, beta_i, delta_i (per phase), gamma_C]
%             alpha_i = 1/L_i
%             beta_i  = RL_i/L_i
%             delta_i = VD_i/L_i
%             gamma_C = 1/C
%
%  I_load is NOT in theta — it is estimated separately via the Vout
%  innovation, then fed back into the WyNDA Vout prediction each step.
%
%  Vout prediction (with I_load correction):
%    Vout(k+1|k) = Vout(k|k) + Ts*gamma_C*I_sw - Ts*I_load_est/C
%
%  Vout innovation encodes the I_load error:
%    innov_Vout ≈ Ts*(I_load_est - I_load_true) / C
%  Therefore:
%    I_load_est += -gamma_Iload * (C_est/Ts) * innov_Vout

n_x     = n_phases + 1;
n_theta = 3*n_phases + 1;   % alpha_i, beta_i, delta_i per phase + gamma_C

%% Nominal values & initial theta
L_nom  = 80e-3;
C_nom  = 12e-3;
RL_nom = 10e-3;
VD_nom = 0.8;

theta_hat = zeros(n_theta, 1);
for i = 1:n_phases
    theta_hat(3*(i-1)+1) = 1/L_nom;        % alpha_i
    theta_hat(3*(i-1)+2) = RL_nom/L_nom;   % beta_i
    theta_hat(3*(i-1)+3) = VD_nom/L_nom;   % delta_i
end
theta_hat(3*n_phases+1) = 1/C_nom;         % gamma_C

%% Bounds in theta-space
tol_L  = 0.1;
L_min  = L_nom*(1-tol_L);  L_max  = L_nom*(1+tol_L);
RL_min = 0.001;             RL_max = 0.1;
VD_min = 0.7;               VD_max = 1.5;
tol_C  = 0.1;
C_min  = C_nom*(1-tol_C);  C_max  = C_nom*(1+tol_C);
P_max  = 15000;             Vout_ref = 96;

theta_min = zeros(n_theta,1);
theta_max = zeros(n_theta,1);
for i = 1:n_phases
    theta_min(3*(i-1)+1) = 1/L_max;       theta_max(3*(i-1)+1) = 1/L_min;
    theta_min(3*(i-1)+2) = RL_min/L_max;  theta_max(3*(i-1)+2) = RL_max/L_min;
    theta_min(3*(i-1)+3) = VD_min/L_max;  theta_max(3*(i-1)+3) = VD_max/L_min;
end
theta_min(3*n_phases+1) = 1/C_max;        theta_max(3*n_phases+1) = 1/C_min;

%% WyNDA tuning
sigma_IL   = 0.1;    % [A]  — tune to sensor noise
sigma_Vout = 0.1;    % [V]
Rx = diag([sigma_IL^2 * ones(1,n_phases), sigma_Vout^2]);

% Rtheta_obs: small → trust parameter-driven model; large → distrust it
Rtheta_obs = 1e-4 * eye(n_x);

Px     = 0.1 * eye(n_x);
Ptheta = 0.1 * eye(n_theta);

% Block-diagonal Gamma: each state couples ONLY to its own parameters.
% Avoids cross-contamination between inductor and capacitor equations.
% Γ evolves naturally toward the right coupling structure via Eq (19).
Gamma = zeros(n_x, n_theta);
for i = 1:n_phases
    Gamma(i, 3*(i-1)+(1:3)) = 1e-4;   % inductor row i → alpha,beta,delta_i
end
Gamma(n_phases+1, 3*n_phases+1) = 1e-4;  % Vout row → gamma_C only

lambda_x     = 0.995;
lambda_theta = 0.999;

x_hat = zeros(n_x, 1);   % warm-started on first sample

%% I_load estimator (separate from WyNDA)
% Driven by the Vout component of the WyNDA innovation:
%   innov_Vout = Vout_meas - Vout_pred
%              ≈ Ts*(I_load_est - I_load_true)/C  (if pred model is right)
% So: I_load_est -= gamma_Iload * (C_est/Ts) * innov_Vout
I_load_est   = 0;
gamma_Iload  = 5e-2;   % dimensionless: larger → faster but noisier
Iload_min    = 0;
Iload_max    = P_max;

%% Stage 2: load decomposition MRAC (unchanged from previous version)
G_est    = 0;   Idc_est  = 0;   CP_est   = 0;
gamma_G   = 1e-1;
gamma_Idc = 1e1;
gamma_CP  = 1e-1;
G_min_s2  = 0;  G_max_s2   = P_max/(Vout_ref^2);
Idc_min_s2 = 0; Idc_max_s2 = P_max/Vout_ref;
CP_min_s2  = 0; CP_max_s2  = P_max;

%% Storage
N = max(size(IL_m));
t = (0:N-1)*Ts;

x_hat_store    = zeros(n_x, N);
theta_store    = zeros(n_theta, N);
innov_store    = zeros(n_x, N);

L_store        = zeros(n_phases, N);
RL_store       = zeros(n_phases, N);
VD_store       = zeros(n_phases, N);
C_store        = zeros(1, N);
Iload_store    = zeros(1, N);

IL_dot_store   = zeros(n_phases, N);
Vout_dot_store = zeros(1, N);

G_store        = zeros(1, N);
Idc_store      = zeros(1, N);
CP_store       = zeros(1, N);
r_load_store   = zeros(1, N);

Pin_store  = zeros(1, N);
Pout_store = zeros(1, N);
PR_store   = zeros(1, N);
PD_store   = zeros(1, N);
PL_store   = zeros(1, N);
PC_store   = zeros(1, N);

%% ── Main Loop ─────────────────────────────────────────────────────────────
for k = 1:N

    %% Current measurements
    y         = [IL_m(k,:)'; Vout_m(k)];
    IL_meas   = IL_m(k,:)';
    Vout_last = [Vout_last(2:end); Vout_m(k)];
    Vin_last       = [Vin_last(2:end); Vin_m(k)];

    Vin = c_val*Vin_last;
    Vout_meas = c_val*Vout_last;

    %% Warm-start state on first sample
    if k == 1
        x_hat = y;
    end

    %% Build Ψ(y(k), u(k))  [n_x × n_theta]
    % Ts is folded in so θ stays in physical units (no /Ts scaling needed)
    Psi = zeros(n_x, n_theta);
    for i = 1:n_phases
        col = 3*(i-1)+1;
        Psi(i, col)   =  Ts * (Vin - Vout_meas*(1-u(i)));   % alpha_i
        Psi(i, col+1) =  Ts * (-IL_meas(i));                  % beta_i
        Psi(i, col+2) =  Ts * (-(1-u(i)));                    % delta_i
    end
    I_sw = sum(IL_meas .* (1-u));
    Psi(n_phases+1, 3*n_phases+1) = Ts * I_sw;               % gamma_C (no I_load term)

    %% ── WyNDA Eqs (11–14): Gain computation ──────────────────────────────
    Kx       = Px / (Px + Rx);
    Omega    = Gamma * Ptheta * Gamma' + Rtheta_obs;
    Ktheta   = Ptheta * Gamma' / Omega;
    Gamma_kk = (eye(n_x) - Kx) * Gamma;

    %% ── WyNDA Eqs (9–10): State and parameter update ─────────────────────
    innov      = y - x_hat;          % y - x_hat_pred (x_hat already has I_load correction)

    x_hat_kk  = x_hat    + (Kx + Gamma_kk*Ktheta) * innov;   % Eq (9)
    theta_kk  = theta_hat - Ktheta * innov;                    % Eq (10)
    theta_kk  = min(max(theta_kk, theta_min), theta_max);

    %% Recover C from current theta (needed for I_load update)
    gC_k  = max(theta_kk(3*n_phases+1), 1e-9);
    C_est = 1/gC_k;

    %% ── I_load update from Vout innovation ───────────────────────────────
    % x_hat_pred for Vout (previous step) = x_hat_kk_prev + Ts*gC*I_sw_prev
    %                                       - Ts*I_load_est_prev/C_prev
    % → innov_Vout ≈ Ts*(I_load_est - I_load_true)/C
    % → I_load_est -= gamma * (C/Ts) * innov_Vout   to drive error to zero
    innov_Vout = innov(n_phases+1);
    I_load_est = I_load_est - gamma_Iload * (C_est/Ts) * innov_Vout;
    I_load_est = min(max(I_load_est, Iload_min), Iload_max);

    %% ── WyNDA Eqs (15–19): Prediction for next step ──────────────────────
    x_hat_pred    = x_hat_kk + Psi * theta_kk;
    % Explicit I_load correction on Vout prediction:
    % without this, WyNDA sees a persistent Vout bias and wrongly adapts gamma_C
    x_hat_pred(n_phases+1) = x_hat_pred(n_phases+1) - Ts * I_load_est / C_est;

    theta_hat_pred = theta_kk;   % parameters constant between steps

    Px_pred     = (1/lambda_x)     * (eye(n_x)     - Kx)     * Px;
    Ptheta_pred = (1/lambda_theta) * (eye(n_theta) - Ktheta*Gamma) * Ptheta;
    Gamma_pred  = Gamma_kk - Psi;

    %% Advance to next step
    x_hat     = x_hat_pred;
    theta_hat = theta_hat_pred;
    Px        = Px_pred;
    Ptheta    = Ptheta_pred;
    Gamma     = Gamma_pred;

    %% Recover all physical parameters from theta_kk
    for i = 1:n_phases
        ai = max(theta_kk(3*(i-1)+1), 1e-9);
        bi = theta_kk(3*(i-1)+2);
        di = theta_kk(3*(i-1)+3);
        L_store(i,k)  = 1/ai;
        RL_store(i,k) = bi/ai;
        VD_store(i,k) = di/ai;
    end
    C_store(k)     = C_est;
    Iload_store(k) = I_load_est;

    %% Model-predicted derivatives (physical, from recovered params)
    IL_dot_model = zeros(n_phases,1);
    for i = 1:n_phases
        ai = theta_kk(3*(i-1)+1);
        bi = theta_kk(3*(i-1)+2);
        di = theta_kk(3*(i-1)+3);
        IL_dot_model(i) = ai*(Vin - Vout_meas*(1-u(i))) ...
                        - bi*IL_meas(i) - di*(1-u(i));
    end
    Vout_dot_model = (I_sw - I_load_est) / C_est;  % KCL

    IL_dot_store(:,k)  = IL_dot_model;
    Vout_dot_store(k)  = Vout_dot_model;
    innov_store(:,k)   = innov;

    %% Power conservation
    Pin_store(k)   = Vin * sum(IL_meas);
    Pout_store(k)  = Vout_meas * I_load_est;
    PR_store(k)    = sum(RL_store(:,k) .* IL_meas.^2);
    PD_store(k)    = sum(VD_store(:,k) .* IL_meas .* (1-u));
    PL_store(k)    = sum(L_store(:,k)  .* IL_meas .* IL_dot_model);
    PC_store(k)    = C_est * Vout_meas * Vout_dot_model;

    %% Stage 2: load model decomposition (unchanged)
    r_load  = I_load_est - (G_est*Vout_meas + Idc_est + CP_est/(Vout_meas+1e-3));
    G_est   = G_est   + Ts * gamma_G   * r_load * Vout_meas;
    Idc_est = Idc_est + Ts * gamma_Idc * r_load;
    CP_est  = CP_est  + Ts * gamma_CP  * r_load / (Vout_meas+1e-3);
    G_est   = min(max(G_est,   G_min_s2),  G_max_s2);
    Idc_est = min(max(Idc_est, Idc_min_s2), Idc_max_s2);
    CP_est  = min(max(CP_est,  CP_min_s2),  CP_max_s2);

    G_store(k)      = G_est;
    Idc_store(k)    = Idc_est;
    CP_store(k)     = CP_est;
    r_load_store(k) = r_load;

    %% Store
    x_hat_store(:,k) = x_hat_kk;
    theta_store(:,k)  = theta_kk;
end

%% ── PLOTS ─────────────────────────────────────────────────────────────────

%% Figure 1 – Inductor currents: measured vs WyNDA state estimate
figure('Name','Phase Currents');
for i = 1:n_phases
    subplot(n_phases,1,i); hold on; grid on;
    plot(t, IL_m(:,i),        'LineWidth',2, 'DisplayName','Measured');
    plot(t, x_hat_store(i,:), 'LineWidth',2, 'DisplayName','WyNDA state');
    ylabel(sprintf('I_{L%d} (A)',i)); legend('Location','best');
    if i==1; title('Inductor Currents: Measured vs WyNDA'); end
end
xlabel('Time (s)');

%% Figure 2 – Output voltage
figure('Name','Output Voltage'); hold on; grid on;
plot(t, Vout_m,                      'LineWidth',2, 'DisplayName','Measured');
plot(t, x_hat_store(n_phases+1,:),   'LineWidth',2, 'DisplayName','WyNDA state');
ylabel('V_{out} (V)'); xlabel('Time (s)');
title('Output Voltage: Measured vs WyNDA'); legend('Location','best');

%% Figure 3 – Input voltage
figure('Name','Input Voltage'); hold on; grid on;
plot(t, Vin_m, 'LineWidth',2);
ylabel('V_{in} (V)'); xlabel('Time (s)');
title('Input Voltage (Measurement)'); grid on;

%% Figure 4 – Inductor current derivatives
figure('Name','Inductor Current Derivatives');
for i = 1:n_phases
    subplot(n_phases,1,i); hold on; grid on;
    plot(t, IL_dot_store(i,:), 'LineWidth',2);
    ylabel(sprintf('dI_{L%d}/dt (A/s)',i));
    if i==1; title('Inductor Current Derivatives (from WyNDA theta)'); end
    grid on;
end
xlabel('Time (s)');

%% Figure 5 – Output voltage derivative
figure('Name','Output Voltage Derivative'); hold on; grid on;
plot(t, Vout_dot_store, 'LineWidth',2);
ylabel('dV_{out}/dt (V/s)'); xlabel('Time (s)');
title('Output Voltage Derivative (KCL from WyNDA)');

%% Figure 6 – WyNDA innovation per state
figure('Name','WyNDA Innovation');
for i = 1:n_phases
    subplot(n_phases+1, 1, i); hold on; grid on;
    plot(t, innov_store(i,:), 'LineWidth',1.5);
    yline(0,'k--');
    ylabel(sprintf('e_{IL%d} (A)',i));
    if i==1; title('WyNDA Innovation  y(k) - x-hat(k|k-1)'); end
end
subplot(n_phases+1, 1, n_phases+1); hold on; grid on;
plot(t, innov_store(n_phases+1,:), 'LineWidth',1.5);
yline(0,'k--');
ylabel('e_{Vout} (V)'); xlabel('Time (s)');

%% Figure 7 – I_load estimate vs Stage 2 reconstruction
figure('Name','Load Current');
subplot(2,1,1); hold on; grid on;
plot(t, Iload_store, 'LineWidth',2, 'DisplayName','I_{load} (WyNDA innov)');
plot(t, G_store.*Vout_m' + Idc_store + CP_store./(Vout_m'+1e-3), ...
    '--','LineWidth',1.5,'DisplayName','Stage 2 reconstruction');
ylabel('I_{load} (A)'); legend('Location','best');
title('Load Current: WyNDA Estimate vs Stage 2 Reconstruction');

subplot(2,1,2); hold on; grid on;
plot(t, r_load_store, 'LineWidth',1.5);
yline(0,'k--');
ylabel('r_{load} (A)'); xlabel('Time (s)');
title('Stage 2 Decomposition Residual');

%% Figure 8 – Inductance L
figure('Name','Inductance');
for i = 1:n_phases
    subplot(n_phases,1,i); hold on; grid on;
    plot(t, L_store(i,:)*1e3, 'LineWidth',2);
    yline(L_nom*1e3,'k--','Nominal');
    ylabel(sprintf('L_%d (mH)',i));
    if i==1; title('Inductance Estimates (WyNDA)'); end
end
xlabel('Time (s)');

%% Figure 9 – Inductor resistance RL
% FIX: use sprintf with '\\Omega' so sprintf produces literal \Omega for TeX
figure('Name','Inductor Resistance');
for i = 1:n_phases
    subplot(n_phases,1,i); hold on; grid on;
    plot(t, RL_store(i,:)*1e3, 'LineWidth',2);
    ylabel(sprintf('R_{L%d} (m\\Omega)',i));   % <-- double backslash fixes warning
    if i==1; title('Inductor Resistance Estimates (WyNDA)'); end
end
xlabel('Time (s)');

%% Figure 10 – Diode voltage VD
figure('Name','Diode Voltage');
for i = 1:n_phases
    subplot(n_phases,1,i); hold on; grid on;
    plot(t, VD_store(i,:), 'LineWidth',2);
    ylabel(sprintf('V_{D%d} (V)',i));
    if i==1; title('Diode Voltage Estimates (WyNDA)'); end
end
xlabel('Time (s)');

%% Figure 11 – Capacitance C
figure('Name','Capacitance'); hold on; grid on;
plot(t, C_store*1e3, 'LineWidth',2);
yline(C_nom*1e3,'k--','Nominal');
ylabel('C (mF)'); xlabel('Time (s)');
title('Capacitance Estimate (WyNDA)');

%% Figure 12 – Load parameters (Stage 2)
figure('Name','Load Parameters');
subplot(3,1,1); hold on; grid on;
plot(t, G_store,   'LineWidth',2);
ylabel('G_{load} (S)'); title('Load Parameters (Stage 2 MRAC)');
subplot(3,1,2); hold on; grid on;
plot(t, Idc_store, 'LineWidth',2);
ylabel('I_{dc,load} (A)');
subplot(3,1,3); hold on; grid on;
plot(t, CP_store,  'LineWidth',2);
ylabel('P_{cp,load} (W)'); xlabel('Time (s)');

%% Figure 13 – Power Conservation
figure('Name','Power Conservation');
subplot(2,1,1); hold on; grid on;
plot(t, Pin_store, 'k',  'LineWidth',2,   'DisplayName','P_{in}');
plot(t, Pout_store + PR_store + PD_store + PL_store + PC_store, ...
    'r--','LineWidth',1.5,'DisplayName','P_{out}+losses');
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