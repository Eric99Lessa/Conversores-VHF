clear; clc;

syms r [3 1]
syms s

poly_cubic = (s - r(1))*(s - r(2))*(s - r(3));
poly_cubic = simplify(poly_cubic);
poly_cubic = collect(poly_cubic, s);
coeffs_cubic = coeffs(poly_cubic, s, 'All');

syms tau wn zeta positive

poly_dyn = (s + 1/tau)*(s^2 + 2*wn*zeta*s + wn^2);
poly_dyn = simplify(poly_dyn);
poly_dyn = collect(poly_dyn, s);
coeffs_dyn = coeffs(poly_dyn, s, 'All');

% r1*r2 = wn^2
% r1 + r2 = 2*zeta*wn
% r3 = 1/tau