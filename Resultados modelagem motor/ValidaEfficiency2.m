clc
clear all
close all

rpm=2500:100:8000;

power=1:1:22;

[RPM POWER]=meshgrid(rpm,power);
Eff=zeros(length(power),length(rpm));

for i=1:length(power)
    for j=1:length(rpm)
        Eff(i,j)=Efficiency2(rpm(j),power(i));
    end
end

figure(1)
contour(RPM,POWER,Eff)
colorbar
xlabel('Rotação (rpm)')
ylabel('Potência (cv)')
zlabel('Eficiência Energética (%)')

figure(2)
surf(RPM,POWER,Eff)
xlabel('Rotação (rpm)')
ylabel('Potência (cv)')
zlabel('Eficiência Energética (%)')

figure(3)
subplot(2,1,1)
surf(RPM,POWER,Eff)
xlabel('Rotação (rpm)')
ylabel('Potência (cv)')
zlabel('Eficiência Energética (%)')
subplot(2,1,2)
contour(RPM,POWER,Eff)
colorbar
xlabel('Rotação (rpm)')
ylabel('Potência (cv)')
zlabel('Eficiência Energética (%)')

max(max(Eff))