clear; close all; clc

%% Dados calibracao sensor de corrente
data_corrente = [0	0.063	-0.44	-0.02;
                0.106	0.17	-0.33	0.023;
                0.212	0.31	-0.21	0.087;
                0.309	0.4	-0.11	0.21;
                0.422	0.5	0.01	0.42;
                0.507	0.576	0.1	0.58;
                0.606	0.7	0.21	0.71;
                0.713	0.8	0.34	0.78;
                0.816	0.9	0.44	0.8;
                0.903	1.01	0.55	0.83;
                1.024	1.12	0.6	0.93;
                1.128	1.23	0.66	1.14;
                1.203	1.29	0.74	1.3;
                1.306	1.42	0.84	1.46;
                1.425	1.52	0.98	1.56;
                1.506	1.61	1.06	1.59;
                1.613	1.74	1.18	1.62;
                1.711	1.84	1.29	1.68;
                1.808	1.93	1.4	1.82;
                1.953	2.08	1.56	2.06;
                2.037	2.16	1.64	2.22;
                2.169	2.29	1.77	2.35;
                2.267	2.4	1.82	2.39;
                2.394	2.55	1.96	2.44;
                2.53	2.68	2.1	2.57;
                2.675	2.81	2.26	2.8;
                2.77	2.92	2.37	2.99;
                2.884	3.04	2.48	3.13;
                3.05	3.21	2.63	3.19;
                3.163	3.35	2.73	3.22;
                3.275	3.44	2.83	3.27];
data_corrente(:, 2) = data_corrente(:, 2) - data_corrente(1, 2);
data_corrente(:, 1:2) = data_corrente(:, 1:2)*20; % 20 foi o numero de espiras usados no teste

data_corrente = array2table(data_corrente, 'VariableNames', {'Corrente Multímetro', 'Corrente Osciloscópio', 'Tensão Rm', 'Tensão Micro'});

%% Dados calibracao sensor de tensao
data_tensao = [0.162	-0.411	0.0434;
               5.04	-0.199	0.261;
               10.04	0.0174	0.464;
               14.95	0.227	0.621;
               20.03	0.465	0.827;
               25.03	0.608	1.04;
               30.06	0.783	1.27;
               35	0.997	1.48;
               40.08	1.22	1.7;
               45	1.44	1.92;
               50	1.68	2.14;
               54.9	1.83	2.36;
               60	2.05	2.5];

data_tensao = array2table(data_tensao, 'VariableNames', {'Tensão Multímetro', 'Tensão Rm', 'Tensão Micro'});

%% Dados ADC
data_ADC_1 = [0.140 7;
             0.160 30;
             0.180 54;
             0.200 78;
             0.250 138;
             0.300 196;
             0.400 316;
             0.501 434;
             0.600 560;
             0.800 798;
             1.000 1047;
             1.200 1292;
             1.400 1533;
             1.600 1778;
             1.800 2020;
             2.000 2261;
             2.200 2505;
             2.300 2630;
             2.350 2691;
             2.400 2755;
             2.450 2819;
             2.480 2862;
             2.500 2887;
             2.520 2914;
             2.550 2957;
             2.570 2986;
             2.600 3028;
             2.700 3193;
             2.800 3374;
             2.900 3568;
             2.950 3680;
             3.000 3791;
             3.050 3907;
             3.100 4030;
             3.120 4079;
             3.130 4095];
data_ADC_2 = [128	1;
              153	18;
              176	46;
              199	73;
              244	127;
              275	163;
              356	261;
              398	311;
              455	377;
              548	490;
              630	591;
              676	647;
              750	735;
              930	933;
              950	980;
              1025	1077;
              1092	1156;
              1162	1241;
              1250	1350;
              1340	1459;
              1401	1535;
              1522	1682;
              1617	1801;
              1709	1908;
              1870	2107;
              1927	2172;
              2080	2356;
              2167	2464;
              2234	2543;
              2371	2717;
              2504	2887;
              2635	3084;
              2804	3370;
              2905	3573;
              3015	3820;
              3105	4027;
              3125	4080];
data_ADC_2(:, 1) = data_ADC_2(:, 1)/1000;

data_ADC = sort([data_ADC_1; data_ADC_2]);
data_ADC = array2table(data_ADC, 'VariableNames', {'Tensão de Entrada', 'Digital Reading'});

%% Fit dados corrente
corrente_mult = data_corrente.("Corrente Multímetro");
corrente_oscil = data_corrente.("Corrente Osciloscópio");
tensao_micro = data_corrente.("Tensão Micro");

[a_1, tensao_micro_fit_i1, R_sqr_i1] = linear_regression(corrente_mult, tensao_micro);
K_corrente_1 = 1/a_1;

[a_2, tensao_micro_fit_i2, R_sqr_i2] = linear_regression(corrente_oscil, tensao_micro);
K_corrente_2 = 1/a_2;

%% Plot fit corrente
i = 1;
figure(i)
subplot(1, 2, 1)
scatter(corrente_mult, tensao_micro, 'LineWidth', 1.5); hold on;
plot(corrente_mult, tensao_micro_fit_i1, 'LineWidth', 1.5);
grid on; grid minor;
legend('Dados', sprintf('Regressão Multímetro - R² = %.4f', R_sqr_i1), Location='best')
xlabel("Corrente (A)");
ylabel("Tensão Micro (V)");
title("Regressão linear - Sensor de Corrente")

subplot(1, 2, 2)
scatter(corrente_oscil, tensao_micro, 'LineWidth', 1.5); hold on;
plot(corrente_oscil, tensao_micro_fit_i2, 'LineWidth', 1.5);
grid on; grid minor;
legend('Dados', sprintf('Regressão Osciloscópio - R² = %.4f', R_sqr_i2), Location='best')

xlabel("Corrente (A)");
ylabel("Tensão Micro (V)");
title("Regressão linear - Sensor de Corrente")
i = i + 1;

%% Fit dados tensao
tensao_mult = data_tensao.("Tensão Multímetro");
tensao_micro = data_tensao.("Tensão Micro");

[a, tensao_micro_fit, R_sqr] = linear_regression(tensao_mult, tensao_micro);
K_tensao = 1/a;

%% Plot fit tensao
figure(i)
scatter(tensao_mult, tensao_micro, 'LineWidth', 1.5); hold on;
plot(tensao_mult, tensao_micro_fit, 'LineWidth', 1.5);
grid on; grid minor;
legend('Dados', sprintf('Regressão - R² = %.4f', R_sqr), Location='best')

xlabel("Tensão Medida (V)");
ylabel("Tensão Micro (V)");
title("Regressão linear - Sensor de Tensão")
i = i + 1;

%% Divide dados do ADC em 2 retas
adc_reading = data_ADC.("Digital Reading");
tensao_adc = data_ADC.("Tensão de Entrada");

V_linmax = 2.6; % Tensao maxima para saída linear
idx = find(tensao_adc>=V_linmax, 1, 'first');
tensao_adc_1 = tensao_adc(1:idx);
tensao_adc_2 = tensao_adc(idx:end);
adc_reading_1 = adc_reading(1:idx);
adc_reading_2 = adc_reading(idx:end);

%% Regressao linear e Fit primeira curva

% [a, tensao_adc_1_fit_reg, R_sqr_1] = linear_regression(adc_reading_1, tensao_adc_1);

poly_1 = polyfit(adc_reading_1, tensao_adc_1, 1);

tensao_adc_1_fit_poly = polyval(poly_1, adc_reading_1);
SStot = sum((tensao_adc_1 - mean(tensao_adc_1)).^2);      % Total Sum-Of-Squares
SSres = sum((tensao_adc_1 - tensao_adc_1_fit_poly).^2);        % Residual Sum-Of-Squares
R_sqr_2 = 1 - SSres/SStot; 

%% Desloca segunda curva

adc_aux = adc_reading_2 - adc_reading_1(end);
tensao_adc_aux = tensao_adc_2 - tensao_adc_1_fit_poly(end);

[a, tensao_adc_aux_fit, R_sqr_aux] = linear_regression(adc_aux, tensao_adc_aux);
poly_2 = [a, -a*adc_reading_2(1) +  tensao_adc_1_fit_poly(end)];
tensao_adc_2_fit = tensao_adc_aux_fit + tensao_adc_1_fit_poly(end);

tensao_adc_fit = [tensao_adc_1_fit_poly; tensao_adc_2_fit(2:end)];

SStot = sum((tensao_adc - mean(tensao_adc)).^2);      % Total Sum-Of-Squares
SSres = sum((tensao_adc - tensao_adc_fit).^2);        % Residual Sum-Of-Squares
R_sqr = 1 - SSres/SStot; 

%% Plot fit ADC
figure(i)
scatter(adc_reading, tensao_adc, 'LineWidth', 1.5); hold on;
plot(adc_reading_1, tensao_adc_1_fit_poly, 'LineWidth', 1.5);
plot(adc_reading_2, tensao_adc_2_fit, 'LineWidth', 1.5);
grid on; grid minor;
legend('Dados',  ...
    'Fit 1ª curva', 'Fit 2ª curva', ...
    Location='best')

xlabel("Leitura ADC");
ylabel("Tensão ADC (V)");
title(sprintf("Calibração ADC - R² = %f", R_sqr))
i = i + 1;

%% Teste

adc_reading(idx)
format long
poly_2
poly_1
function adc_voltage = binaryToVoltage(adc_value)
    if (adc_value == 0)
        adc_voltage = 0;
    elseif (adc_value > 3028)
        adc_voltage = 0.000486251139392 * adc_value + 1.149508344943664;
    else
        adc_voltage = 0.000818852818358 * adc_value + 0.142390461037213;
    end
end

N = 100;
adc_values = linspace(0, 4096, N)';
adc_voltages = zeros(N, 1);
for n = 1:N
    adc_voltages(n) = binaryToVoltage(adc_values(n));
end

figure(i)
scatter(adc_values, adc_voltages);