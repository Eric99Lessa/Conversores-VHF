clear

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
syms Vout Vin R_ind L C I_load real

%% LHS
% O lado esquerdo das equações do modelo é composto pelas equações das
% correntes em cada indutor e tensão no capacitor de saída

for i = 1:n_eq
    if i <= n_phases
        In_L(i) = sum(xn((1:n_states_eq) + n_states_eq*(i - 1)).*basis);
        LHS(i) = diff(In_L(i), t);
    else
        Vout = sum(xn((1:n_states_eq) + n_states_eq*(i - 1)).*basis);
        LHS(i) = diff(Vout, t);
    end
end

%% RHS
% O lado direito das equações inclui a influencia da tensão de entrada,
% resistencia e indutancia dos indutores em cada fase

for i = 1:n_phases
    % se u = 0
    VLi_u0 = Vin - R_ind*In_L(i) - Vout;
    IC_u0 = In_L(i);

    % se u = 1
    VLi_u1 = Vin - R_ind*In_L(i);
    IC_u1 = 0;

    VLi_u = VLi_u0 + (VLi_u1 - VLi_u0)*d(i);
    IC_u(i) = IC_u0 + (IC_u1 - IC_u0)*d(i);

    RHS(i) = VLi_u/L;
end
RHS(end) = (sum(IC_u) + I_load)/C;

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
solution = solve(eqs, Xn_dot);

% Convert struct to symbolic vector
X_dot = struct2array(solution);
X_dot = X_dot(:);

% Define state vector X and input u
X = Xn;
U = d; % Assume Vin is the input

% Define Hamiltonian
H = (L*sum(X(1:3).^2) + C*X(4)^2)/2;
jac_H = jacobian(H, X); jac_H = jac_H(:);

%% Express system in port-Hamiltonian form: x_dot = (J - R)*dH/dx + g*u + xi
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

R = (jac_X + jac_X')/2;
J = (jac_X - jac_X')/2;

%% Closed-loop system

syms IL_ref [n_phases 1] real
syms Vout_ref real

% Define Hamiltonian
Hd = (L*sum((X(1:3) - IL_ref).^2) + C*(X(4) - Vout_ref)^2)/2;
jac_Hd = jacobian(Hd, X); jac_Hd = simplify(jac_Hd(:));

Jd = J;
Rd = R;

LHS_IDA = (Jd - Rd)*jac_Hd;
exp = LHS_IDA - (J - R)*jac_H - xi;
