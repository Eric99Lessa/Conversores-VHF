clc
clear all
close all

theta=10:5:90;
ntheta=length(theta);
RPM=1800:100:9000;
nRPM=length(RPM);

Torque=zeros(ntheta,nRPM);
Power=zeros(ntheta,nRPM);


for i=1:ntheta
    for j=1:nRPM
        [Torque(i,j),Power(i,j)]=CalcICE(RPM(j),theta(i));
    end
    
    figure(1)
    hold on
    plot(RPM,Torque(i,:),'Linewidth',1)
    grid on
    xlabel('Rotação (rpm)')
    ylabel('Torque (kgfm)')
    hold off
    
    figure(2)
    hold on
    plot(RPM,Power(i,:),'Linewidth',1)
    grid on
    xlabel('Rotação (rpm)')
    ylabel('Potência (cv)')
    hold off
      
end
[RPM_3D,theta_3D]=meshgrid(RPM,theta);

figure(3)
contour(RPM_3D,theta_3D,Torque)
xlabel('Rotação (rpm)')
ylabel('\theta (grau)')
zlabel('Torque (kgfm)')

figure(4)
surf(RPM_3D,theta_3D,Torque)
xlabel('Rotação (rpm)')
ylabel('\theta (grau)')
zlabel('Torque (kgfm)')

figure(5)
contour(RPM_3D,theta_3D,Power)
xlabel('Rotação (rpm)')
ylabel('\theta (grau)')
zlabel('Potência (cv)')

figure(6)
surf(RPM_3D,theta_3D,Power)
xlabel('Rotação (rpm)')
ylabel('\theta (grau)')
zlabel('Potência (cv)')

figure(7)
subplot(2,1,1)
surf(RPM_3D,theta_3D,Torque)
xlabel('Rotação (rpm)')
ylabel('\theta (grau)')
zlabel('Torque (kgfm)')
subplot(2,1,2)
surf(RPM_3D,theta_3D,Power)
xlabel('Rotação (rpm)')
ylabel('\theta (grau)')
zlabel('Potência (cv)')