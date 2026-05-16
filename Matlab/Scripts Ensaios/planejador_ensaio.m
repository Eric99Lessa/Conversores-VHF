clear; clc; close all;

%% 
P_des = 5e3;
R_load = [0.29 0.46 0.65 0.81 1.02 1.21 1.41 1.58 1.77 1.95 2.16 2.32 2.51 ...
    2.73 2.96 3.08 3.28 3.45 3.64 3.83 4.05 4.19 4.36 4.53 4.71 4.92];

%%

Vout = sqrt(P_des.*R_load);
Iout = Vout./R_load;

Vin = 50:10:100; Vin = Vin(:);

[R_grid, Vin_grid] = meshgrid(R_load, Vin);
[Vout_grid, ~] = meshgrid(Vout, Vin);

duty_grid = 1 - Vin_grid./Vout_grid;

%% 

figure(1)

surf(Vin_grid, R_grid, 100*duty_grid);
xlabel('Tensão de Entrada (V)');
ylabel('Resistência Carga (ohm)');
zlabel('Duty Cycle (%)');