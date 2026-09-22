%%
clear; close all; clc;

addpath(pwd + "/Simulink/");
% dados_conversor;
dictionaries;

%% Controle
fs = 1e3;
Ts = 1/fs;

%% Simulation - Teste 3 fases
voltage_source = "DC";
converter_mode = 0;
control_str = "Open"; % "IDAPBC" "Krasovskii" "CascadeModelBased" "CascadeModelFree" "Open"
control_mode = map_control(control_str);
var_LV = 0.1^2;
var_HV = 0.1^2;
var_IL = 0.1^2;
n_phases = 3;
f_cutoff = 100;
Vout_ref = 24;
decim = 10;
Tend = 3;

R_load = [0 1; Tend 1];
Idc_load = [0 0; 1 0; 1.1 15; Tend 15];
% Idc_load = [0 0; Tend 0];
CP_load = [0 0; 2 0; 2.1 400; Tend 400];
% CP_load = [0 0; Tend 0];

%%
out = sim("boost_control.slx");

%% 
tc = out.tc;
td = out.td;
P_est = out.P_est;
Pin = P_est(:, 1);
Pout = P_est(:, 2);
PR = P_est(:, 3);
PD = P_est(:, 4);
PL = P_est(:, 5);
PC = P_est(:, 6);
I_load_est = out.y_est(:, end);

%% Plots Controle

% Plot Tensão
id_fig = 1;
figure(id_fig)
subplot(2, 1, 1)
plot(tc, out.LV, 'lineWidth', 2); hold on;
plot(tc, out.HV, 'lineWidth', 2); hold on;
yline(Vout_ref, '--', 'LineWidth', 2);
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Tensão (V)', 'FontSize', 14);
if converter_mode == 0
    legend('Entrada', 'Saída', 'Referência', 'FontSize', 14, 'Location', 'best');
elseif converter_mode == 1
    legend('Saída', 'Entrada', 'Referência', 'FontSize', 14, 'Location', 'best');
else
    legend('Bateria', 'Barramento', 'Referência', 'FontSize', 14, 'Location', 'best');
end
title('Tensões de Entrada e Saída', 'FontSize', 18);

subplot(2, 1, 2)
if converter_mode == 0
    plot(td, out.D_bot, 'lineWidth', 2);
else
    plot(td, out.D_top, 'LineWidth', 2);
end
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Duty Cycle (%)', 'FontSize', 14);
legend('U', 'V', 'W', 'FontSize', 14, 'Location', 'best');
title('Duty Cycle (%)', 'FontSize', 18);

% Plot Corrente
id_fig = id_fig + 1;
figure(id_fig)
subplot(2, 1, 1)
plot(tc, out.IL, 'lineWidth', 2); hold on;
plot(td, out.IL_eq, '--', 'lineWidth', 2); hold on;
grid on; grid minor;
xlabel("Tempo (s)");
ylabel("Corrente (A)");
legend("IL1", "IL2", "IL3", "Referencia", 'Location','best')
title('Correntes nos indutores', 'FontSize', 18);

subplot(2, 1, 2)
if converter_mode == 0
    plot(td, out.D_bot, 'lineWidth', 2);
else
    plot(td, out.D_top, 'LineWidth', 2);
end
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Duty Cycle (%)', 'FontSize', 14);
legend('U', 'V', 'W', 'FontSize', 14, 'Location', 'best');
title('Duty Cycle (%)', 'FontSize', 18);

%% Plots Observador
% Plot Correntes
id_fig = id_fig + 1;
figure(id_fig)
subplot(3, 1, 1)
plot(td, out.IL_m(:, 1), 'lineWidth', 2); hold on;
plot(td, out.y_est(:, 1), 'lineWidth', 2); hold on;
title('Corrente nos indutores', 'FontSize', 18);
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Corrente (A)', 'FontSize', 14);
legend('Medida', 'Estimativa', 'FontSize', 14, 'Location', 'best');

subplot(3, 1, 2)
plot(td, out.IL_m(:, 2), 'lineWidth', 2); hold on;
plot(td, out.y_est(:, 2), 'lineWidth', 2); hold on;
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Corrente (A)', 'FontSize', 14);
legend('Medida', 'Estimativa', 'FontSize', 14, 'Location', 'best');

subplot(3, 1, 3)
plot(td, out.IL_m(:, 3), 'lineWidth', 2); hold on;
plot(td, out.y_est(:, 3), 'lineWidth', 2); hold on;
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Corrente (A)', 'FontSize', 14);
legend('Medida', 'Estimativa', 'FontSize', 14, 'Location', 'best');

% Plot Derivada Correntes
id_fig = id_fig + 1;
figure(id_fig)
subplot(3, 1, 1)
plot(td, out.y_dot_filt(:, 1), 'lineWidth', 2); hold on;
plot(td, out.y_dot_model(:, 1), 'lineWidth', 2); hold on;
title('Derivada da corrente nos indutores', 'FontSize', 18);
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Derivada da Corrente (A/s)', 'FontSize', 14);
legend('Estimativa', 'Modelo', 'FontSize', 14, 'Location', 'best');

subplot(3, 1, 2)
plot(td, out.y_dot_filt(:, 2), 'lineWidth', 2); hold on;
plot(td, out.y_dot_model(:, 2), 'lineWidth', 2); hold on;
title('Derivada da corrente nos indutores', 'FontSize', 18);
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Derivada da Corrente (A/s)', 'FontSize', 14);
legend('Estimativa', 'Modelo', 'FontSize', 14, 'Location', 'best');

subplot(3, 1, 3)
plot(td, out.y_dot_filt(:, 3), 'lineWidth', 2); hold on;
plot(td, out.y_dot_model(:, 3), 'lineWidth', 2); hold on;
title('Derivada da corrente nos indutores', 'FontSize', 18);
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Derivada da Corrente (A/s)', 'FontSize', 14);
legend('Estimativa', 'Modelo', 'FontSize', 14, 'Location', 'best');

% Plot Tensões
id_fig = id_fig + 1;
figure(id_fig)
subplot(2, 1, 1)
plot(td, out.LV_m, 'lineWidth', 2); hold on;
plot(td, out.y_est(:, 5), 'lineWidth', 2); hold on;
title('Tensão de Entrada', 'FontSize', 18);
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Tensão (V)', 'FontSize', 14);
legend('Medida', 'Estimativa', 'FontSize', 14, 'Location', 'best');

subplot(2, 1, 2)
plot(td, out.HV_m, 'lineWidth', 2); hold on;
plot(td, out.y_est(:, 4), 'lineWidth', 2); hold on;
title('Tensão de Saída', 'FontSize', 18);
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Tensão (V)', 'FontSize', 14);
legend('Medida', 'Estimativa', 'FontSize', 14, 'Location', 'best');

% Plot Derivada Tensao de saida
id_fig = id_fig + 1;
figure(id_fig)
plot(td, out.y_dot_filt(:, 4), 'lineWidth', 2); hold on;
plot(td, out.y_dot_model(:, 4), 'lineWidth', 2); hold on;
title('Derivada da Tensão de saída', 'FontSize', 18);
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Derivada da Tensão (V/s)', 'FontSize', 14);
legend('Estimativa', 'Modelo', 'FontSize', 14, 'Location', 'best');

% Potência de entrada e saída
id_fig = id_fig + 1;
figure(id_fig)
subplot(3, 1, 1)
plot(td, Pin, 'LineWidth', 2.5); hold on;
plot(td, Pout, 'LineWidth', 2.5); hold on;
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Potência (W)', 'FontSize', 14);
legend('Entrada', 'Saída', 'FontSize', 14, 'Location', 'best');
title('Potências na Entrada e Saída', 'FontSize', 18)

subplot(3, 1, 2)
plot(td, 100*Pout./Pin, 'LineWidth', 2.5); hold on;
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Eficiência (-)', 'FontSize', 14);

subplot(3, 1, 3)
plot(td, PR, 'LineWidth', 2.5); hold on;
plot(td, PD, 'LineWidth', 2.5); hold on;
plot(td, PL, 'LineWidth', 2.5); hold on;
plot(td, PC, 'LineWidth', 2.5); hold on;
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Potência (W)', 'FontSize', 14);
legend('Resistência', 'Diodo', 'Indutor', 'Capacitor', 'FontSize', 14, 'Location', 'best');

% Corrente da carga
id_fig = id_fig + 1;
figure(id_fig)
plot(tc, out.I_load, 'lineWidth', 2); hold on;
plot(td, I_load_est, 'lineWidth', 2);
grid on; grid minor;
xlabel("Tempo (s)");
ylabel("Corrente (A)");
legend("Saída", "Saída estimada", 'Location','best')
title('Correntes de Saída', 'FontSize', 18);

%% Plots Parametros

id_fig = id_fig + 1;
figure(id_fig)
subplot(2, 1, 1)
plot(td, out.res_IL, 'lineWidth', 2); hold on;
title('Resíduo', 'FontSize', 18);
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Resíduo (-)', 'FontSize', 14);
legend('IL_{1}', 'IL_{2}', 'IL_{3}', 'FontSize', 14, 'Location', 'best');

subplot(2, 1, 2)
plot(td, out.res_P, 'lineWidth', 2); hold on;
title('Resíduo da Potência', 'FontSize', 18);
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Resíduo (W)', 'FontSize', 14);

id_fig = id_fig + 1;
figure(id_fig)
subplot(2, 1, 1)
plot(td, out.RL_est(:, 1:3), 'lineWidth', 2); hold on;
title('Resistência dos indutores', 'FontSize', 18);
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('R_{L} (ohm)', 'FontSize', 14);
legend('L_{1}', 'L_{2}', 'L_{3}', 'FontSize', 14, 'Location', 'best');

subplot(2, 1, 2)
plot(td, out.VD_est(:, 1:3), 'lineWidth', 2); hold on;
title('Tensão nos diodos', 'FontSize', 18);
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('V_{D} (V)', 'FontSize', 14);
legend('L_{1}', 'L_{2}', 'L_{3}', 'FontSize', 14, 'Location', 'best');

id_fig = id_fig + 1;
figure(id_fig)
plot(td, out.res_I_load, 'lineWidth', 2); hold on;
title('Resíduo carga', 'FontSize', 18);
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Resíduo (-)', 'FontSize', 14);

id_fig = id_fig + 1;
figure(id_fig)
subplot(3, 1, 1)
plot(td, out.load_est(:, 1), 'lineWidth', 2); hold on;
title('Parametros Modelo Carga', 'FontSize', 18);
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('G_{load} (S)', 'FontSize', 14);

subplot(3, 1, 2)
plot(td, out.load_est(:, 2), 'lineWidth', 2); hold on;
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Idc_{load} (A)', 'FontSize', 14);

subplot(3, 1, 3)
plot(td, out.load_est(:, 3), 'lineWidth', 2); hold on;
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('CP_{load} (W)', 'FontSize', 14);

%%
LV_m = out.LV_m;
HV_m = out.HV_m;
IL_m = out.IL_m;
LV = out.LV;
HV = out.HV;
IL = out.IL;
I_load = out.I_load;
td = out.td;
tc = out.tc;

save("boost_data", "IL_m", "HV_m", "LV_m", "IL", "HV", "LV", "I_load", "td", "Ts", "tc");