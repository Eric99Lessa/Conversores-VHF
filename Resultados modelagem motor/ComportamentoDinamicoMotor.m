clc
clear all
close all

load ('Dados_MCI.mat')

dt=0.001;
t=0:dt:10;
nd=length(t);
theta1=11*ones(1,round(nd/4));
theta2=15*ones(1,round(nd/4));
%theta3=19*ones(1,round(nd/4));
theta3=75*ones(1,round(nd/4));
theta4=11*ones(1,round(nd/4));
theta=[theta1 theta2 theta3 theta4];
theta(nd)=11;

delay_iter=round(Delay/dt);
rpm=RPM_idle*ones(1,nd);
for i=delay_iter+1:nd
    Torque=CalcICE(rpm(i-1),theta(i-delay_iter))*9.8;
    wp=Torque/J;
    w=(rpm(i-1)*pi/30)+wp*dt;
    rpm(i)=w*30/pi;
end

figure(1)
plot(t, rpm, 'Linewidth', 1)
grid on
xlabel('Tempo (s)')
ylabel('Rotação (rpm)')

figure(2)
plot(t, theta, 'Linewidth', 1)
grid on
xlabel('Tempo (s)')
ylabel('\theta (grau)')




