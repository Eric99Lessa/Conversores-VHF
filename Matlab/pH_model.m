clear; clc;

syms t T D real

% Fundamental frequency
omega0 = 2*pi/T;

n_phases = 3; % Number of phases
order_approx = 0; % Approximation order
n_states_eq = 1 + 2*order_approx; % Number of states per equation
n_eq = n_phases + 1; % 1 equation for capacitor and 1 for each inductor

%% PWM
% Define the piecewise function for the PWM signal u(t)
syms d [n_phases 1]
u = piecewise(0 <= t & t < D*T, 1, D*T <= t & t < T, 0);

% Initialize basis
basis = sym(ones(n_states_eq, 1));
u_approx = (1/T)*int(1, t, 0, D*T);

% Preallocate symbolic array for phase-shifted signals
syms un un_approx

% Generate phase-shifted PWM signals
for i = 1:n_phases
    t_delay = (i - 1)*T/n_phases;
    un(i) = subs(u, t, t - t_delay);
    un_approx(i) = subs(u_approx, t, t - t_delay);
end

% Plot the original PWM signals
plota = false; % Bool para plot
if plota
    plot_comp_approx(un, un_approx);
end

%% States variables
n_states = n_states_eq*n_eq; 
syms xn xn_dot [n_states 1]

for i = 1:n_states
    syms(sprintf('x%d(t)', i)) %declare each element in the array as a single symbolic function
    xn(i) = symfun(eval(sprintf('x%d(t)', i)), t); %declare each element to a symbolic "handle"
    xn_dot(i) = diff(xn(i), t);
end

syms In_L IC_u [n_phases 1]
syms LHS RHS [n_eq 1]
syms VC Vin R_ind L C I_load real
assumeAlso(R_ind > 0);
assumeAlso(L > 0);
assumeAlso(C > 0);

%% LHS
% O lado esquerdo das equações do modelo é composto pelas equações das
% correntes em cada indutor e tensão no capacitor de saída

for i = 1:n_eq
    if i <= n_phases
        In_L(i) = sum(xn((1:n_states_eq) + n_states_eq*(i - 1)).*basis);
        LHS(i) = L*diff(In_L(i), t);
    else
        VC = sum(xn((1:n_states_eq) + n_states_eq*(i - 1)).*basis);
        LHS(i) = C*diff(VC, t);
    end
end

%% RHS
% O lado direito das equações inclui a influencia da tensão de entrada,
% resistencia e indutancia dos indutores em cada fase

for i = 1:n_phases
    % se u = 0
    VLi_u0 = Vin - R_ind*In_L(i) - VC;
    IC_u0 = In_L(i);

    % se u = 1
    VLi_u1 = Vin - R_ind*In_L(i);
    IC_u1 = 0;

    VLi_u = VLi_u0*(1 - d(i)) + VLi_u1*d(i);
    IC_u(i) = IC_u0*(1 - d(i)) + IC_u1*d(i);

    RHS(i) = VLi_u;
end
RHS(end) = sum(IC_u) - I_load;

%% Substitui funcoes do tempo
syms Xn Xn_dot [n_states 1] real

old_vars = [xn_dot; xn];
new_vars = [Xn_dot; Xn];

for i = 1:n_eq
    LHS(i) = subsExpression(LHS(i), old_vars, new_vars);
    LHS(i) = collect(LHS(i), basis);
    RHS(i) = subsExpression(RHS(i), old_vars, new_vars);
    RHS(i) = collect(RHS(i), basis);
end

eqs = LHS(:) == RHS(:);

% Solve for X_dot
[solution] = solve(eqs, Xn_dot, 'ReturnConditions', true);
solution = rmfield(solution, intersect(fieldnames(solution), {'parameters','conditions'}));

% Convert struct to symbolic vector
X_dot = struct2array(solution);
X_dot = X_dot(:);

%% Express system in port-Hamiltonian form: x_dot = (J - R)*dH/dx + g*u + xi
% Prepare states and input
X = Xn;
U = d;

% Define Hamiltonian and jacobian
Q = [sqrt(L)*eye(n_phases) zeros(n_phases, 1);
     zeros(1, n_phases) sqrt(C)];
H = 0.5*(Q*X).'*(Q*X);
jac_H = jacobian(H, X); jac_H = jac_H(:);

% Input Matrix
g_x = jacobian(X_dot, U);

X_dot_aux = simplify(X_dot - g_x*U);

% External
xi = subs(X_dot_aux, X, zeros(size(X)));

X_dot_aux = simplify(X_dot_aux - xi);

jac_X = jacobian(X_dot_aux, X);

for i = 1:size(jac_X, 2)
    jac_X(:, i) = jac_X(:, i)/(jac_H(i)/X(i));
end

R = -(jac_X + jac_X')/2; % R é estrita positiva
J = (jac_X - jac_X')/2;

% X_dot reescrito
% test = X_dot - ((J - R)*jac_H + g_x*U + xi);
% simplify(test)
X_dot = (J - R)*jac_H + g_x*U + xi;

%% Closed-loop system

syms Vout_ref IL_ref IL_ref_dot Vout_ref_dot real
syms err [(n_phases + 1) 1] real
syms Kr Kj real

% Define closed-loop Hamiltonian
Hd = 0.5*(Q*err).'*(Q*err);
jac_Hd = jacobian(Hd, err); jac_Hd = simplify(jac_Hd(:));
X_ref = [IL_ref*ones(n_phases, 1); Vout_ref];
X_ref_dot = [IL_ref_dot*ones(n_phases, 1); Vout_ref_dot];
jac_Hd = subs(jac_Hd, err, X - X_ref);

Jc = Kj*J;
Rc = Kr*R;
Jd = simplify(J + Jc); assumeAlso(Kj > 0);
Rd = simplify(R + Rc); assumeAlso(Kr > 0);

% Nota: jac_H pode ser escrito como diag([L c])*X, diag([L C]) = D_lc

D_lc = Q.^2;

%% Lei de Controle

% Agora precisamos achar a equação de D que garante as raízes nos lugares
% desejados

% Igualando as derivadas em malha aberta e fechada, isolando o duty cycle:
% g_x*U = (Jd - Rd)*jac_Hd + X_ref_dot - (J - R)*jac_H-xi
% (J - R)*(jac_Hd - jac_H) + (Jc - Rc)*jac_Hd + X_ref_dot - xi

% g_x*U = (J - R)*D_lc*X_ref + (Jc - Rc)*D_lc*(X - X_ref) + X_ref_dot - xi

% como g(x) não é quadrada, precisamos usar a pseudo inversa, que é:
% inv(g_x'*g_x)*g_x' e pode ser obtida pela função pinv

% Jd = subs(Jd, Kj, Kj_sol);
% Rd = Kr*R;

expr_ma = (J - R)*D_lc*X + g_x*d + xi;

expr_mf = (Jd - Rd)*D_lc*(X - X_ref) + X_ref_dot;
expr_mf = subs(expr_mf, Vout_ref_dot, 0);

g_perp = null(g_x.')';
g_perp*g_x

res = simplify(g_perp*(expr_ma - expr_mf), 1000)
res_reg = simplify(subs(res, [IL_ref_dot, Vout_ref_dot], [0,0]));

% Isolando g_x*d
LHS = g_x*d;
RHS = expr_mf - expr_ma + LHS;

pinv_g_x = simplify(collect(pinv(g_x)));
d_sol = pinv_g_x*RHS;

% d_sol = subs(d_sol, Kj, Kj_sol);

syms Vout real
syms IL [1 n_phases] real
d_sol = subs(d_sol, Xn, [IL Vout]');
[d_num, d_den] = numden(d_sol);

d_num = simplify(collect(d_num, [C^2 L^2]));

matlabFunction(d_num, 'File', 'function_d_num');
matlabFunction(d_den, 'File', 'function_d_den');
% matlabFunction(Kr_sol, 'File', 'function_Kr');
% matlabFunction(Kj_sol, 'File', 'function_Kj');

%% Plota raizes reais em função do ganho Kr

A_mf = (Jd - Rd)*D_lc;

roots = eig(A_mf);
n_roots = length(roots);

% Vamos separar as raízes que são sempre reais das que podem ser complexas

isreal_eig = false(n_eq, 1);
for i = 1:n_eq
    isreal_eig(i) = isreal(roots(i));
end

real_roots = simplify(roots(isreal_eig));
conjugate_roots = simplify(roots(~isreal_eig));

Kj_range = linspace(0, 3, 100);
Kr_range = linspace(0, 20, 100);
params.L = 80e-3;
params.C = 12000e-3;
params.R_ind = 1e-3;

real_roots_Kr = subs(real_roots, L, 80e-3);
real_roots_Kr = subs(real_roots_Kr, R_ind, 1e-1);
real_roots_vec = double(subs(real_roots_Kr, Kr, Kr_range));

figure(1)
title("Lugar das Raízes reais")
scatter3(real(real_roots_vec), imag(real_roots_vec), Kr_range);
axis equal
grid on
xlabel("Re")
ylabel("Im")
view(2)
% plot_root_locus_2gains_from_roots(roots, Kj_range, Kr_range, params)

% Separa numerador e denominador da expressão, pegando somente a parte
% dentro da raiz quadrada
[num, den] = numden((conjugate_roots(1) - conjugate_roots(2))/2);

% Eleva ao quadrado para sumir com a raiz e acha o valor de Kj para que a
% expressão dentro da raiz seja nula - Amortecimento crítico
% Nota que Kj depende de Kr, dessa maneira só precisamos definir 1 dos
% ganhos
[Kj_sol, params, conds] = solve(num^2 == 0, Kj, 'ReturnConditions', true);
Kj_sol = simplify(Kj_sol);

% Assumindo a condição de amortecimento crítico obtida as raízes viram:

conj_roots_d = subs(conjugate_roots, Kj, Kj_sol);

roots_d = [real_roots; conj_roots_d];
syms tau real

[Kr_sol, params, conds] = solve(-tau == real_roots(1), Kr, 'ReturnConditions', true);