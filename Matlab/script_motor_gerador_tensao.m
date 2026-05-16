clear all; close all; clc;

%%
addpath(pwd + "/Simulink/");
addpath(pwd + "/Modelagem motor/");

%% 

rpm_idle = 1800;
rpm_max = 8400;
rpms = linspace(rpm_idle, rpm_max, 10);
len_rpms = length(rpms);
theta_idle = 11;
theta_max = 90;
thetas = linspace(30, theta_max, 5);
len_thetas = length(thetas);

torques = zeros(len_rpms, len_thetas);
potencias = zeros(len_rpms, len_thetas);
potencias_ind = zeros(len_rpms, len_thetas);

for i_rpm = 1:len_rpms
    for i_theta = 1:len_thetas
        [torques(i_rpm, i_theta), potencias(i_rpm, i_theta), potencias_ind(i_rpm, i_theta)] = ...
            CalcICE(rpms(i_rpm), thetas(i_theta));
    end
end

potencias = potencias*735;
potencias_ind = potencias_ind*735;

%% Gerador
K_e = 5.4e-3; % Relação FEM/rotacao
p = 5; % Numero de pares de pólos
R_ger = 1.94; % mOhm - Conferir unidade no simulink
L_ger = 77; % uH - conferir unidade no simulink

%% Retificador trifásico
V_fwd = 1.46; % em Volts e I_fwd = 150A
R_on = (1.46-1.13)/(150 - 50); % Ohm - on resistance
G_off = 100/1200; % uS - off Conductance
%% Capacitor
Cout = 12000e-6;

%% Simulação
Rs_carga = 1000; %[0.1 1 2];
len_Rs = length(Rs_carga);
% rpms = [6000 7000 8000 8400];
% len_rpms = length(rpms);

Tend = 1;

for i_rpm = 1:len_rpms
    rpm = rpms(i_rpm);
    for i_R = 1:len_Rs
        R_carga = Rs_carga(i_R);

        out = sim("simulacao_gerador.slx");

        figure(1)
        plot(out.tout, out.Vout, 'lineWidth', 2, 'DisplayName', num2str(rpm)); hold on;
    end
end

legend('show'); 