function [Torque,Power_net,Power_ind]=CalcICE(rpmi,thetai)
    load('Dados_MCI.mat')
    theta=[11 15 30 50 70 90];
    w=[0 0];
    max_w=20;
    
    if rpmi<RPM_idle
        rpmi=RPM_idle;
    end
    
    if rpmi>RPM_max
        Torque=-1*(b0+b1*rpmi*(pi/30))/9.8;
    elseif thetai<=theta(1)
        Torque=polyval(p_Torque_11,rpmi);
    elseif thetai>theta(1)&&thetai<=theta(2)
        Torque1=polyval(p_Torque_11,rpmi);
        w(1)=min(1./(abs(thetai-theta(1))),max_w);
        Torque2=polyval(p_Torque_15,rpmi);
        w(2)=min(1./(abs(thetai-theta(2))),max_w);
        Torque=(Torque1*w(1)+Torque2*w(2))/sum(w);
    elseif thetai>theta(2)&&thetai<=theta(3)
        Torque1=polyval(p_Torque_15,rpmi);
        w(1)=min(1./(abs(thetai-theta(2))),max_w);
        Torque2=polyval(p_Torque_30,rpmi);
        w(2)=min(1./(abs(thetai-theta(3))),max_w);
        Torque=(Torque1*w(1)+Torque2*w(2))/sum(w);
    elseif thetai>theta(3)&&thetai<=theta(4)
        Torque1=polyval(p_Torque_30,rpmi);
        w(1)=min(1./(abs(thetai-theta(3))),max_w);
        Torque2=polyval(p_Torque_50,rpmi);
        w(2)=min(1./(abs(thetai-theta(4))),max_w);
        Torque=(Torque1*w(1)+Torque2*w(2))/sum(w);
    elseif thetai>theta(4)&&thetai<=theta(5)
        Torque1=polyval(p_Torque_50,rpmi);
        w(1)=min(1./(abs(thetai-theta(4))),max_w);
        Torque2=polyval(p_Torque_70,rpmi);
        w(2)=min(1./(abs(thetai-theta(5))),max_w);
        Torque=(Torque1*w(1)+Torque2*w(2))/sum(w);
    elseif thetai>theta(5)&&thetai<theta(6)
        Torque1=polyval(p_Torque_70,rpmi);
        w(1)=min(1./(abs(thetai-theta(5))),max_w);
        Torque2=polyval(p_Torque_90,rpmi);
        w(2)=min(1./(abs(thetai-theta(6))),max_w);
        Torque=(Torque1*w(1)+Torque2*w(2))/sum(w);
    elseif thetai>=theta(6)
        Torque=polyval(p_Torque_90,rpmi);
    end
    
    Power_net=((Torque*9.8)*(rpmi*(pi/30)))/735;
    
    Power_ind=Power_net+((b0+b1*rpmi*pi/30)*rpmi*pi/30)/735; %Potência gerada pelo MCI
end