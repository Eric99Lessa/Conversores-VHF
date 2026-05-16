clear; clc; close all;
%% Abre arquivo

% filename = "teste_1901.csv";
% filename = "teste_1901_1.csv";
% filename = "teste_1901_2.csv";
% filename = "teste_1901_3.csv";
% filename = "teste_1901_4.csv";
filename = "teste_2101_4.csv";
% filename = "teste_2201_2.csv";
% filename = "teste_2301_1.csv";
R_ensaio = 4.97; %R_ensaio = 2.5;
C_out = 831e-6;
T = readmatrix(filename);

%% Divide em arrays de acordo com o nome da coluna
% ordem: t seq Vin Vout IL1 IL2 IL3 ILout du dv dw
% idx:   1  2   3   4    5   6   7    8   9  10 11
t = T(:, 1);
t = t - t(1);
Ts = mean(diff(t));
Vin = T(:, 3);
Vout = T(:, 4);
IL = T(:, 5:7);
Iout = T(:, 8);
duty = T(:, 9:11)/1000; % vem em permille

Vout_dot = [0; diff(Vout)./Ts];
% Vout_dot = lowpass(Vout_dot, 0.5, 1/Ts);
Iout_est = sum(IL.*(1 - duty), 2) - C_out*Vout_dot;

%% Calcula potencias
Pin = sum(Vin.*IL, 2);
Pout = (Vout.^2)./R_ensaio;
Pout_est = Vout.*Iout_est;

%% Plota tensões e duty cycles
figure(1)

subplot(2, 1, 1)
plot(t, Vin, 'LineWidth', 2.5); hold on;
plot(t, Vout, 'LineWidth', 2.5);
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Tensão (V)', 'FontSize', 14);
legend('Entrada', 'Saída', 'FontSize', 14, 'Location', 'best');
title('Tensões de Entrada e Saída', 'FontSize', 18);

subplot(2, 1, 2)
plot(t, 100*duty, 'LineWidth', 2.5);
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Duty Cycle (%)', 'FontSize', 14);
legend('U', 'V', 'W', 'FontSize', 14, 'Location', 'best');
title('Duty Cycle (%)', 'FontSize', 18);

%% Plota Correntes
figure(2)

subplot(3, 1, 1)
plot(t, IL, 'LineWidth', 2.5); hold on;
% plot(t, Iout, 'LineWidth', 2);
plot(t, Iout_est, 'LineWidth', 2.5);
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Corrente (A)', 'FontSize', 14);
legend('U', 'V', 'W', 'Saída estimada', 'FontSize', 14, 'Location', 'best');
title('Correntes nos Indutores e Saída', 'FontSize', 18);

subplot(3, 1, 2)
plot(t, C_out*Vout_dot, 'LineWidth', 2.5);
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Corrente (A)', 'FontSize', 14);
title('Corrente estimada no capacitor de saída', 'FontSize', 18);

subplot(3, 1, 3)
plot(t, 100*duty, 'LineWidth', 2.5);
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Duty Cycle (%)', 'FontSize', 14);
legend('U', 'V', 'W', 'FontSize', 14, 'Location', 'best');
title('Duty Cycle (%)', 'FontSize', 18);

%% Potência de entrada e saída
figure(3)

subplot(2, 1, 1)
plot(t, Pin, 'LineWidth', 2.5); hold on;
plot(t, Pout, 'LineWidth', 2.5); hold on;
plot(t, Pout_est, 'LineWidth', 2.5); hold on;
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Potência (W)', 'FontSize', 14);
legend('Entrada', 'Saída', 'Saída Estimada', 'FontSize', 14, 'Location', 'best');
title('Potências na Entrada e Saída', 'FontSize', 18)

subplot(2, 1, 2)
plot(t, 100*duty, 'LineWidth', 2.5);
grid on; grid minor;
ax = gca;
ax.FontSize = 14;
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('Duty Cycle (%)', 'FontSize', 14);
legend('U', 'V', 'W', 'FontSize', 14, 'Location', 'best');
title('Duty Cycle (%)', 'FontSize', 18);