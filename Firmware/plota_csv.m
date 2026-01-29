function [] = plota_csv(filename, C_out, R_ensaio)
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
    plot(t, Vin, 'LineWidth', 2); hold on;
    plot(t, Vout, 'LineWidth', 2);
    grid on; grid minor;
    xlabel('Tempo (s)');
    ylabel('Tensão (V)');
    legend('Entrada', 'Saída');
    title('Tensões de Entrada e Saída');
    
    subplot(2, 1, 2)
    plot(t, 100*duty, 'LineWidth', 2);
    grid on; grid minor;
    xlabel('Tempo (s)');
    ylabel('Duty Cycle (%)');
    legend('U', 'V', 'W');
    title('Duty Cycle (%)');
    
    %% Plota Correntes
    figure(2)
    
    subplot(3, 1, 1)
    plot(t, IL, 'LineWidth', 2); hold on;
    % plot(t, Iout, 'LineWidth', 2);
    plot(t, Iout_est, 'LineWidth', 2);
    grid on; grid minor;
    xlabel('Tempo (s)');
    ylabel('Corrente (A)');
    legend('U', 'V', 'W', 'Saída estimada');
    title('Correntes nos Indutores e Saída');
    
    subplot(3, 1, 2)
    plot(t, C_out*Vout_dot, 'LineWidth', 2);
    grid on; grid minor;
    xlabel('Tempo (s)');
    ylabel('Corrente (A)');
    title('Corrente estimada no capacitor de saída');
    
    subplot(3, 1, 3)
    plot(t, 100*duty, 'LineWidth', 2);
    grid on; grid minor;
    xlabel('Tempo (s)');
    ylabel('Duty Cycle (%)');
    legend('U', 'V', 'W');
    title('Duty Cycle (%)');
    
    %% Potência de entrada e saída
    figure(3)
    subplot(2, 1, 1)
    plot(t, Pin, 'LineWidth', 2); hold on;
    plot(t, Pout, 'LineWidth', 2); hold on;
    plot(t, Pout_est, 'LineWidth', 2); hold on;
    grid on; grid minor;
    xlabel('Tempo (s)');
    ylabel('Potência (W)');
    legend('Entrada', 'Saída', 'Saída Estimada');
    title('Potências na Entrada e Saída')
    
    subplot(2, 1, 2)
    plot(t, 100*duty, 'LineWidth', 2);
    grid on; grid minor;
    xlabel('Tempo (s)');
    ylabel('Duty Cycle (%)');
    legend('U', 'V', 'W');
    title('Duty Cycle (%)');
end