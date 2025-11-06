clc
clear all
close all

Eff_exp=[
11.75	16.97	16.16	10.16	7.25	15.31	14.16	5.65	10.36	4.20	3.81	5.40	7.67
14.14	28.32	25.70	23.73	22.41	21.42	19.61	19.01	16.46	15.47	12.57	8.27	2.84
13.80	31.79	29.93	28.13	28.20	27.15	25.63	25.96	23.32	23.12	17.14	14.13	10.93
14.03	31.41	31.47	31.26	30.96	31.29	29.30	29.55	27.04	29.07	22.83	18.70	17.98
14.05	31.52	31.67	31.36	31.92	33.22	30.85	30.92	28.34	29.65	24.39	20.58	18.24
14.05	30.39	31.51	30.67	32.24	32.95	31.74	32.26	28.12	28.28	22.95	19.04	15.90
13.79	30.02	30.55	30.45	31.97	33.12	32.11	32.49	27.36	27.21	22.88	19.22	16.19
12.98	28.05	28.62	29.24	30.07	30.98	29.99	30.58	26.59	24.62	21.90	19.51	16.38];

rpm=[2500	3000	3500	4000	4500	5000	5500	6000	6500	7000	7500	8000	8500];

theta=[20 30 40 50 60 70 80 90];

[RPM THETA]=meshgrid(rpm,theta);

figure(1)
contour(RPM,THETA,Eff_exp)
colorbar
xlabel('Rotação (rpm)')
ylabel('\theta (grau)')
zlabel('Eficiência Energética (%)')

figure(2)
surf(RPM,THETA,Eff_exp)
xlabel('Rotação (rpm)')
ylabel('\theta (grau)')
zlabel('Eficiência Energética (%)')

figure(3)
subplot(2,1,1)
surf(RPM,THETA,Eff_exp)
xlabel('Rotação (rpm)')
ylabel('\theta (grau)')
zlabel('Eficiência Energética (%)')
subplot(2,1,2)
contour(RPM,THETA,Eff_exp)
colorbar
xlabel('Rotação (rpm)')
ylabel('\theta (grau)')
zlabel('Eficiência Energética (%)')