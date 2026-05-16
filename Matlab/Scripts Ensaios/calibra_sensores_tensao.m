clear; clc; close all;
plota = true;
testsFolder = getTestsFolder(pwd);

%% 27/01
filename = "teste_2701_2.csv";
C_out = 831e-6;
R_ensaio = 50.16;
T = readmatrix(testsFolder + filename);

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

%% Plota tensões e duty cycles
if plota
    figure(1)
    
    subplot(3, 1, 1)
    plot(t, Vin, 'LineWidth', 2); hold on;
    grid on; grid minor;
    xlabel('Tempo (s)');
    ylabel('Leitura (int)');
    title('Tensão de entrada')
    
    subplot(3, 1, 2)
    plot(t, Vout, 'LineWidth', 2); hold on;
    grid on; grid minor;
    xlabel('Tempo (s)');
    ylabel('Leitura (int)');
    title('Tensão de saída')

    subplot(3, 1, 3)
    plot(t, 100*duty, 'LineWidth', 2); hold on;
    grid on; grid minor;
    xlabel('Tempo (s)');
    ylabel('Duty Cycle (%)');
    legend('U', 'V', 'W');
    title('Duty Cycle (%)');
end

V_mult = [6.31 9.66 14.4 19.53 24.4 29.5 36.5 45.27 51.1 59.9 64.9 75.9 77.6 84.2 88.3 90.5 95.6 99.5 29.5]';

%% Divide ensaio na parte em que o duty cycle começou a ser alterado

idx_change_duty = find(diff(duty(:, 1)));
idx_parte1 = 1:idx_change_duty(1);
idx_parte2 = idx_change_duty(1)+1:idx_change_duty(end);

t_1 = t(1:idx_change_duty(1));
t_2 = t(idx_change_duty(1)+1:end);
Vin_1 = Vin(1:idx_change_duty(1));
Vin_2 = Vin(idx_change_duty(1)+1:end);
Vout_1 = Vout(1:idx_change_duty(1));
Vout_2 = Vout(idx_change_duty(1)+1:end);
V_mult_1 = V_mult(1:6);
V_mult_2 = V_mult(7:end-1);

figure(2)
subplot(2, 1, 1)
plot(t_1, Vin_1, 'lineWidth', 2); hold on;
plot(t_2, Vin_2, 'lineWidth', 2); hold on;

subplot(2, 1, 2)
plot(t_1, Vout_1, 'lineWidth', 2); hold on;
plot(t_2, Vout_2, 'lineWidth', 2); hold on;

%%

len_step_dot = 50;
tau_step = 0.5;
ts_step_dot = t(1:len_step_dot);
step_dot = -exp(-ts_step_dot/tau_step)*(-1/tau_step);
    
[t_1_filt_Vin, Vin_1_filt, means_Vin_1] = filt_step_conv(t_1, Vin_1, step_dot);
[t_1_fill_Vout, Vout_1_filt, means_Vout_1] = filt_step_conv(t_1, Vout_1, step_dot);

figure(3)

subplot(2, 1, 1)
plot(t_1, Vin_1, 'lineWidth', 2); hold on;
plot(t_1_filt_Vin, Vin_1_filt, 'lineWidth', 2); hold on;

subplot(2, 1, 2)
plot(t_1, Vout_1, 'lineWidth', 2); hold on;
plot(t_1_fill_Vout, Vout_1_filt, 'lineWidth', 2); hold on;

%%

Vin_2_cell = cell(length(idx_change_duty)-1, 1);
Vout_2_cell = cell(length(idx_change_duty)-1, 1);
means_Vin_2 = zeros(length(idx_change_duty)-1, 1);
means_Vout_2 = zeros(length(idx_change_duty)-1, 1);

figure(4)
for i = 1:length(Vin_2_cell)
    Vin_2_cell{i} = Vin(idx_change_duty(i)+1:idx_change_duty(i + 1));
    Vout_2_cell{i} = Vout(idx_change_duty(i)+1:idx_change_duty(i + 1));

    means_Vin_2(i) = mean(rmoutliers(Vin_2_cell{i}));
    means_Vout_2(i) = mean(rmoutliers(Vout_2_cell{i}));

    subplot(2, 1, 1)
    plot(t(idx_change_duty(i)+1:idx_change_duty(i + 1)), Vin_2_cell{i}, 'lineWidth', 2); hold on;
    plot([t(idx_change_duty(i)+1); t(idx_change_duty(i + 1))], [means_Vin_2(i); means_Vin_2(i)], 'lineWidth', 2);

    subplot(2, 1, 2)
    plot(t(idx_change_duty(i)+1:idx_change_duty(i + 1)), Vout_2_cell{i}, 'lineWidth', 2); hold on;
    plot([t(idx_change_duty(i)+1); t(idx_change_duty(i + 1))], [means_Vout_2(i); means_Vout_2(i)], 'lineWidth', 2);
end

means_Vin = [means_Vin_1; means_Vin_2]; means_Vin = [means_Vin(1:end-2); means_Vin(end)];
means_Vout = [means_Vout_1; means_Vout_2]; means_Vout = [means_Vout(1:end-2); means_Vout(end)];

%%
value_adc_limit = 2890;
pp_Vin = polyfit_esp32(means_Vin, V_mult(1:end-1), value_adc_limit);
pp_Vout = polyfit_esp32(means_Vout, V_mult(1:end-1), value_adc_limit);

figure(6)
subplot(2, 1, 1)
scatter(means_Vin, V_mult(1:end-1), 'lineWidth', 2); hold on;
plot(means_Vin, ppval(pp_Vin, means_Vin), 'lineWidth', 2);
subplot(2, 1, 2)
scatter(means_Vout, V_mult(1:end-1), 'lineWidth', 2); hold on;
plot(means_Vout, ppval(pp_Vout, means_Vout), 'lineWidth', 2);

figure(7)

subplot(3, 1, 1)
plot(t, ppval(pp_Vin, Vin), 'LineWidth', 2); hold on;
grid on; grid minor;
xlabel('Tempo (s)');
ylabel('Tensão (V)');
title('Tensão de entrada')

subplot(3, 1, 2)
plot(t, ppval(pp_Vout, Vout), 'LineWidth', 2); hold on;
grid on; grid minor;
xlabel('Tempo (s)');
ylabel('Tensão (V)');
title('Tensão de saída')

subplot(3, 1, 3)
plot(t, 100*duty, 'LineWidth', 2); hold on;
grid on; grid minor;
xlabel('Tempo (s)');
ylabel('Duty Cycle (%)');
legend('U', 'V', 'W');
title('Duty Cycle (%)');

%% 

% testando = 1;
% % figure(1)
% % plot(Vin, 'LineWidth', 2); hold on;
% % plot(Vout, 'LineWidth', 2);
% % grid on; grid minor;
% % xlabel('Tempo (s)');
% % ylabel('Tensão (V)');
% % legend('Entrada', 'Saída');
% % title('Tensões de Entrada e Saída');
% 
% %% 
% % % Ensaio 1: Alimentação com bateria 12 V - duty cycle 0%
% % 
% % Vin_multimetro = 11.71;
% % Vout_multimetro = 11.151;
% % 
% % filename = "teste_2301_3.csv";
% % R_ensaio = 4.97;
% % C_out = 831e-6;
% % T = readmatrix(filename);
% % 
% % %% Divide em arrays de acordo com o nome da coluna
% % % ordem: t seq Vin Vout IL1 IL2 IL3 ILout du dv dw
% % % idx:   1  2   3   4    5   6   7    8   9  10 11
% % t = T(:, 1);
% % t = t - t(1);
% % Ts = mean(diff(t));
% % Vin = T(:, 3);
% % Vout = T(:, 4);
% % IL = T(:, 5:7);
% % Iout = T(:, 8);
% % duty = T(:, 9:11)/1000; % vem em permille
% % 
% % %% Calcula valor medio e ganhos
% % 
% % [K_in, Vin_corr] = corrige_medida(Vin, Vin_multimetro, 2);
% % [K_out, Vout_corr] = corrige_medida(Vout, Vout_multimetro, 2);
% % 
% % %% Plota correção
% % 
% % if plota
% %     figure(1)
% %     subplot(2, 1, 1)
% %     plot(Vin, 'lineWidth', 2); hold on;
% %     % plot(Vin_corr, 'lineWidth', 2); hold on;
% %     plot(Vin_corr, 'lineWidth', 2); hold on;
% %     line([0 length(Vin)], [Vin_multimetro Vin_multimetro], 'lineWidth', 2);
% %     ax = gca;
% %     ax.FontSize = 14;
% %     xlabel('Amostra');
% %     ylabel('Tensão (V)');
% %     legend('Medida', 'Calibração', 'Multímetro');
% %     title('Calibração - Tensão de Entrada', 'FontSize', 18)
% % 
% %     subplot(2, 1, 2)
% %     plot(Vout, 'lineWidth', 2); hold on;
% %     % plot(Vout_corr, 'lineWidth', 2); hold on;
% %     plot(Vout_corr, 'lineWidth', 2); hold on;
% %     line([0 length(Vout)], [Vout_multimetro Vout_multimetro], 'lineWidth', 2);
% %     ax = gca;
% %     ax.FontSize = 14;
% %     xlabel('Amostra');
% %     ylabel('Tensão (V)');
% %     legend('Medida', 'Calibração', 'Multímetro');
% %     title('Calibração - Tensão de Saída', 'FontSize', 18)
% % end
% % 
% %%
% clear; clc; close all;
% plota = true;
% %% Ensaio 2 - Alimentação com fonte de bancada (variável)
% 
% filename = "teste_2301_5.csv";
% R_ensaio = 4.97;
% C_out = 831e-6;
% T = readmatrix(filename);
% 
% %% Divide em arrays de acordo com o nome da coluna
% % ordem: t seq Vin Vout IL1 IL2 IL3 ILout du dv dw
% % idx:   1  2   3   4    5   6   7    8   9  10 11
% 
% t = T(:, 1);
% t = t - t(1);
% Ts = mean(diff(t));
% Vin = T(:, 3);
% Vout = T(:, 4);
% IL1 = T(:, 5);
% IL2 = T(:, 6);
% IL3 = T(:, 7);
% Iout = T(:, 8);
% duty = T(:, 9:11)/1000; % vem em permille
% 
% %% Remove transients e divide sinais para cada um dos 3 ensaios
% 
% threshold_V = 1;
% Vin = filter_transients(Vin);
% Vin_cell = divide_ensaios(Vin, threshold_V);
% Vout = filter_transients(Vout);
% Vout_cell = divide_ensaios(Vout, threshold_V);
% 
% IL1 = filter_transients(IL1);
% IL2 = filter_transients(IL2);
% IL3 = filter_transients(IL3);
% 
% %%
% 
% if plota
%     figure(3)
%     subplot(3, 1, 1)
%     plot(Vin_cell{1}, 'lineWidth', 2); hold on;
%     plot(Vin_cell{2}, 'lineWidth', 2); hold on;
%     plot(Vin_cell{3}, 'lineWidth', 2); hold on;
%     xlabel('Amostra');
%     ylabel('Tensão (V)');
%     legend('Ensaio 1', 'Ensaio 2', 'Ensaio 3');
%     title('Tensão de Entrada');
% 
%     subplot(3, 1, 2)
%     plot(Vout_cell{1}, 'lineWidth', 2); hold on;
%     plot(Vout_cell{2}, 'lineWidth', 2); hold on;
%     plot(Vout_cell{3}, 'lineWidth', 2); hold on;
%     xlabel('Amostra');
%     ylabel('Tensão (V)');
%     legend('Ensaio 1', 'Ensaio 2', 'Ensaio 3');
%     title('Tensão de Saída');
% 
%     subplot(3, 1, 3)
%     plot(IL1, 'lineWidth', 2); hold on;
%     plot(IL2, 'lineWidth', 2); hold on;
%     plot(IL3, 'lineWidth', 2); hold on;
%     xlabel('Amostra');
%     ylabel('Corrente (A)');
%     title('Corrente nos indutores');
%     legend('IL1', 'IL2', 'IL3')
% end
% 
% %%
% 
% idx_ensaio = 1;
% 
% Vin_mult = [10.42 11.93 15.03 18.01;
%             9.93 11.92 15.08 18.01;
%             10.45 12.19 15.07 18.47];
% Vout_mult = [9.731 11.224 14.302 17.244;
%              9.267 11.1 14.367 17.247;
%              9.770 11.5 14.34 17.712];
% 
% IL1_mult = [2.89 3.33 4.18 5.04];
% IL2_mult = [2.73 3.25 4.17 5.03];
% IL3_mult = [2.88 3.38 4.17 5.18];
% 
% [K_in_1, Vin_corr_1] = corrige_medida(Vin_cell{1}, Vin_mult(1, 3));
% [K_in_2, Vin_corr_2] = corrige_medida(Vin_cell{2}, Vin_mult(2, :));
% [K_in_3, Vin_corr_3] = corrige_medida(Vin_cell{3}, Vin_mult(3, :));
% 
% [K_out_1, Vout_corr_1] = corrige_medida(Vout_cell{1}, Vout_mult(1, 3));
% [K_out_2, Vout_corr_2] = corrige_medida(Vout_cell{2}, Vout_mult(2, :));
% [K_out_3, Vout_corr_3] = corrige_medida(Vout_cell{3}, Vout_mult(3, :));
% 
% [K_IL1, IL1_corr] = corrige_medida(IL1, IL1_mult(3), 0.1);
% [K_IL2, IL2_corr] = corrige_medida(IL2, IL2_mult, 0.3);
% [K_IL3, IL3_corr] = corrige_medida(IL3, IL3_mult, 0.1);
% 
% %%
% figure(4)
% 
% subplot(3, 1, 1)
% plot(IL1, 'lineWidth', 2); hold on;
% plot(IL1_corr, 'lineWidth', 2); hold on;
% xlabel('Amostra');
% ylabel('Corrente (A)');
% title('Corrente nos indutores');
% legend('Medida', 'Correção')
% 
% subplot(3, 1, 2)
% plot(IL2, 'lineWidth', 2); hold on;
% plot(IL2_corr, 'lineWidth', 2); hold on;
% xlabel('Amostra');
% ylabel('Corrente (A)');
% title('Corrente nos indutores');
% legend('Medida', 'Correção')
% 
% subplot(3, 1, 3)
% plot(IL3, 'lineWidth', 2); hold on;
% plot(IL3_corr, 'lineWidth', 2); hold on;
% xlabel('Amostra');
% ylabel('Corrente (A)');
% title('Corrente nos indutores');
% legend('Medida', 'Correção')