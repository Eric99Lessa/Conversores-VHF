function Eff=Efficiency2(rpmi,poweri)

[torque,power_inf]=CalcICE(rpmi,11);
[torque,power_sup]=CalcICE(rpmi,90);

if poweri<power_inf
    Eff=0;
elseif poweri>power_sup
    Eff=0;
else
    theta=11:90;
    power=zeros(1,length(theta));
    dist=zeros(1,length(theta));
    for i=1:length(theta)
        [torque,power(i)]=CalcICE(rpmi,theta(i));
        dist(i)=max(abs(poweri-power(i)),0.001);
    end
    [dist_best, imin]=sort(dist);
    thetai=((theta(imin(1))*(1/dist_best(1)))+(theta(imin(2))*(1/dist_best(2))))/((1/dist_best(1))+(1/dist_best(2)));
    Eff=Efficiency(rpmi,thetai);
end
end