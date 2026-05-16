clear; clc; close all;
plota = true;
testsFolder = getTestsFolder(pwd);

%% 27/01
filename = "teste_0202_S2_1.csv";
C_out = 831e-6;
R_ensaio = 50.16;
T = readmatrix(testsFolder + filename);

%% Divide em arrays de acordo com o nome da coluna
% ordem: t seq Vin Vout IL1 IL2 IL3 ILout du dv dw
% idx:   1  2   3   4    5   6   7    8   9  10 11
t = T(:, 1);
dt = diff(t);
idx_intervalo = dt > 1;
dt(idx_intervalo) = mean(dt(~idx_intervalo));
t = cumsum([0; dt]);
Ts = mean(diff(t));
Vin = T(:, 3);
Vout = T(:, 4);
IL = T(:, 5:7);
Iout = T(:, 8);
duty = T(:, 9:11)/1000; % vem em permille

%%

filedata = "Medidas Ensaio IL2.csv";
n_espiras = 20;

data_IL2 = readtable(testsFolder + filedata, 'Delimiter', ',', 'DecimalSeparator', ',');

IL2_osc = n_espiras*table2array(data_IL2(:, 1));
IL2_osc = IL2_osc(IL2_osc > 0);
IL2_mult = n_espiras*table2array(data_IL2(:, 2));
IL2_mult = IL2_mult(IL2_mult > 0);

%%
figure(1)
plot(t, IL(:, 2), 'LineWidth', 2); hold on;
grid on; grid minor;
xlabel('Tempo (s)');
ylabel('Leitura (int)');
title('Corrente Sensor 2')

len_step_dot = 50;
tau_step = 0.5;
ts_step_dot = t(1:len_step_dot);
step_dot = -exp(-ts_step_dot/tau_step)*(-1/tau_step);
    
[t_filt_IL2, IL2_filt, means_IL2] = filt_step_conv(t, IL(:, 2), step_dot);
means_IL2 = means_IL2(means_IL2 > 0);

figure(1)
plot(t_filt_IL2, IL2_filt, 'LineWidth', 2);

%%

p_IL_mult = polyfit(means_IL2, IL2_mult, 1);
p_IL_osc = polyfit(means_IL2, IL2_osc, 1);

figure(6)
plot(means_IL2, polyval(p_IL_mult, means_IL2), 'lineWidth', 2); hold on;
plot(means_IL2, polyval(p_IL_osc, means_IL2), 'lineWidth', 2); hold on;
scatter(means_IL2, IL2_mult); hold on;
scatter(means_IL2, IL2_osc); hold on;

legend('Fit Multimetro', 'Fit Osciloscopio', 'Multimetro', 'Osciloscopio', Location='best')

% Osciloscopio parece mais fidedigno com o esperado do ensaio (15V/5ohms =
% 3 amps, 3amps*20 = 60amps

%% Divide em 2 retas 

value_adc_limit = 2890;
pp_IL2_osc = polyfit_esp32(means_IL2, IL2_osc, value_adc_limit);
pp_IL2_mult = polyfit_esp32(means_IL2, IL2_mult, value_adc_limit);

%% Plot

figure(7)
plot(means_IL2, ppval(pp_IL2_mult, means_IL2), 'lineWidth', 2); hold on;
plot(means_IL2, ppval(pp_IL2_osc, means_IL2), 'lineWidth', 2); hold on;
scatter(means_IL2, IL2_mult); hold on;
scatter(means_IL2, IL2_osc); hold on;

legend('Fit Multimetro', 'Fit Osciloscopio', 'Multimetro', 'Osciloscopio', Location='best')