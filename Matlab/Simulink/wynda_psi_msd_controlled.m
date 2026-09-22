function Psi = wynda_psi_msd_controlled(y, u)
%WYNDA_PSI_MSD_CONTROLLED Basis matrix for controlled mass-spring-damper.
%
%   System:
%       x1_dot = x2
%       x2_dot = a1*x1 + a2*x2 + a3*u
%
%   Discrete model:
%       x(k+1) = x(k) + Psi(y(k),u(k))*theta
%
%   Inputs
%   ------
%   y : 2-by-1 measured state vector [position; velocity]
%   u : scalar control input
%
%   Output
%   ------
%   Psi : 2-by-5 regression matrix

    y1 = y(1);
    y2 = y(2);

    Psi = [y1, y2, 0,  0,  0;
           0,  0,  y1, y2, u];
end