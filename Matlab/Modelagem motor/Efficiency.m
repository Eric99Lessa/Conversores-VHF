function Eff=Efficiency(rpmi,thetai)

Dados=load('Eff_Data.mat');
RPM=Dados.RPM;
dRPM=RPM(2)-RPM(1);
THETA=Dados.theta;
dTHETA=THETA(2)-THETA(1);
EFF=Dados.Eff_exp;
zero_num=0.001;


[vet,RPM_index]=sort(abs(rpmi-RPM));
rpm=[RPM(RPM_index(1)) RPM(RPM_index(2))];

[vet,THETA_index]=sort(abs(thetai-THETA));
theta=[THETA(THETA_index(1)) THETA(THETA_index(2))];

w=zeros(1,4);
cont=0;
Eff=0;
for i=1:2%rpm
    for j=1:2%theta
        cont=cont+1;
        %w(cont)=((max(zero_num,abs(rpm(i)-rpmi)/dRPM))^(-1))+((max(zero_num,abs(theta(i)-thetai)/dTHETA))^(-1));
        w(cont)=1/max(zero_num,sqrt((abs((rpm(i)-rpmi)/dRPM)^2)+(abs((theta(j)-thetai)/dTHETA)^2)));
        Eff=Eff+w(cont)*EFF(THETA_index(j),RPM_index(i));
    end
end
Eff=Eff/sum(w);

end