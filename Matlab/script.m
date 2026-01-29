clear; clc;
%% Parametros do circuito
%% PWM
f = 10e3; % Hz
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

%% Controle
syms L C R_L tau real
Kr = function_Kr(L, R_L, tau);
Kj = function_Kj(C, L, tau);

tau_min_Kr = solve(Kr == 1, tau);
tau_min_Kj = solve(Kj == 1, tau);

%% Capacitores
C_in = 12000e-6; % F - Conferir unidade no simulink
C_in_esr = 29e-3; % ohm
C_out = 12000e-6; % F - Conferir unidade no simulink
C_out_esr = 29e-3; % ohm

%% Indutores
L_ind = 80e-3; % H - Conferir unidade no simulink
R_ind = 1e0; % ohm - Conferir unidade no simulink

tau_min_Kr = double(subs(tau_min_Kr, [L C R_L], [L_ind C_out, R_ind]));
tau_min_Kj = double(subs(tau_min_Kj, [L C R_L], [L_ind C_out, R_ind]));

tau_val = 0.005;
Kr = double(subs(Kr, [L C R_L tau], [L_ind C_out, R_ind tau_val]));
Kj = double(subs(Kj, [L C R_L tau], [L_ind C_out, R_ind tau_val]));
L = L_ind; R_L = R_ind;
Vout_ref = 96;

%% Simulação
Tend = 1;
t_values = linspace(0, Tend, 100);
P_values = 1000*(t_values < 0.25*Tend) ...
    +      2500*(t_values > 0.25*Tend & t_values < 0.5*Tend) ...
    +      5000*(t_values > 0.5*Tend & t_values < 0.75*Tend) ...
    +      7500*(t_values > 0.75*Tend);
R_load = [t_values(:), (Vout_ref.^2)./P_values(:)];
% R_load = [t_values(:) 100*ones(length(t_values), 1)];

% voltage_source = "GeneratorRectified";
voltage_source = "DC";
decim = 100;

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
plot(out.tout, out.I_in, 'lineWidth', 2); hold on;
plot(out.tout, out.I_load, 'lineWidth', 2);
xlabel("Tempo (s)")
ylabel("Corrente (A)")

% id_fig = id_fig + 1;
% figure(id_fig)
% plot(out.tout, out.I_load, 'lineWidth', 2);
% xlabel("Tempo (s)")
% ylabel("Corrente (A)")

% id_fig = id_fig + 1;
% figure(id_fig)
% plot(out.td, out.D, 'lineWidth', 2);
% xlabel("Tempo (s)")
% ylabel("Duty Cycle (%)")