clear; clc;
%% Parametros do circuito
%% PWM
f = 20e3; % Hz
pwm_T = 1/f;
Ts = 1e-3;

%% Gerador
K_e = 5.4e-3; % Relação FEM/rotacao
p = 5; % Numero de pares de pólos
R_ger = 1.94; % mOhm - Conferir unidade no simulink
L_ger = 77; % uH - conferir unidade no simulink

%% Retificador trifásico
V_fwd = 1.46; % em Volts e I_fwd = 150A
R_on = (1.46-1.13)/(150 - 50); % Ohm - on resistance
G_off = 100/1200; % uS - off Conductance

%% Capacitores
C_in = 12000e-6; % F - Conferir unidade no simulink
C_in_esr = 29e-3; % ohm
C_out = 12000e-6; % F - Conferir unidade no simulink
C_out_esr = 29e-3; % ohm

%% Indutores
L = 80e-3; % H - Conferir unidade no simulink
R_L = 1e-3; % ohm - Conferir unidade no simulink

%% Controle
% -R_ind + L*tau > 0
% L*tau > R_ind
% tau > R_ind/L
tau_min = R_L/L;
tau = tau_min;
Kr = function_Kr(L, R_L, tau);
Kj = function_Kj(C_out, Kr, L, R_L);

%% Simulação
Tend = 1;
R_load = 100;
% voltage_source = "GeneratorRectified";
voltage_source = "DC";

out = sim("feedback_control.slx");

%% Plots
id_fig = 1;

figure(id_fig)
subplot(2, 1, 1)
plot(out.tout, out.VC_in, 'lineWidth', 2); hold on;
plot(out.tout, out.VC_out, 'lineWidth', 2);
xlabel("Tempo (s)")
ylabel("Tensão (V)")
legend("Entrada","Saída", 'Location','best')

subplot(2, 1, 2)
plot(out.tout, out.IC_in, 'lineWidth', 2); hold on;
plot(out.tout, out.IC_out, 'lineWidth', 2);
xlabel("Tempo (s)")
ylabel("Corrente (A)")

id_fig = id_fig + 1;
figure(id_fig)
plot(out.tout, out.I_load, 'lineWidth', 2);
xlabel("Tempo (s)")
ylabel("Corrente (A)")

% id_fig = id_fig + 1;
% figure(id_fig)
% plot(out.td, out.D, 'lineWidth', 2);
% xlabel("Tempo (s)")
% ylabel("Duty Cycle (%)")