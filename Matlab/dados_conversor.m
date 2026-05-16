%% Gerador
K_e = 5.4e-3; % Relação FEM/rotacao
p = 5; % Numero de pares de pólos
R_ger = 1.94; % mOhm - Conferir unidade no simulink
L_ger = 77; % uH - conferir unidade no simulink
f_min = 1800*p/60;
f_max = 8400*p/60;

%% Retificador trifásico
V_fwd = 1.46/2; % em Volts e I_fwd = 150A
R_on = (1.46-1.13)/(150 - 50); % Ohm - on resistance
G_off = 100/1200; % uS - off Conductance

%% Capacitores
C_in = 12; % mF - Conferir unidade no simulink
C_in_esr = 29; % mOhm
C_out = 12; % mF - Conferir unidade no simulink
C_out_esr = 29; % mOhm

%% Indutores
L = 80; % mH - Conferir unidade no simulink
R_L = 100; % mOhm - Conferir unidade no simulink

%% PWM frequency
f_pwm = 10e3;
T_pwm = 1/f_pwm;