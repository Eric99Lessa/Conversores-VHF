clc
clear all
close all

rpm=2500:100:8500;

theta=20:2.5:90;

[RPM, THETA]=meshgrid(rpm,theta);
Eff=zeros(length(theta),length(rpm));

for i=1:length(theta)
    for j=1:length(rpm)
        Eff(i,j)=Efficiency(rpm(j),theta(i));
    end
end

figure(1)
contour(RPM,THETA,Eff)
colorbar
xlabel('Rotação (rpm)')
ylabel('\theta (grau)')
zlabel('Eficiência Energética (%)')

figure(2)
surf(RPM,THETA,Eff)
xlabel('Rotação (rpm)')
ylabel('\theta (grau)')
zlabel('Eficiência Energética (%)')

figure(3)
subplot(2,1,1)
surf(RPM,THETA,Eff)
xlabel('Rotação (rpm)')
ylabel('\theta (grau)')
zlabel('Eficiência Energética (%)')
subplot(2,1,2)
contour(RPM,THETA,Eff)
colorbar
xlabel('Rotação (rpm)')
ylabel('\theta (grau)')
zlabel('Eficiência Energética (%)')

max(max(Eff))