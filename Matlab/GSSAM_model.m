clear

syms t T D real

% Fundamental frequency
omega0 = 2*pi/T;

n_phases = 3; % Number of phases
order_approx = 5; % Approximation order
n_states_eq = 1 + 2*order_approx; % Number of states per equation
n_eq = n_phases + 1; % 1 equation for capacitor and 1 for each inductor

%% PWM
% Define the piecewise function for the PWM signal u(t)
u = piecewise(0 <= t & t < D*T, 1, D*T <= t & t < T, 0);

% Initialize basis
basis = sym(ones(n_states_eq, 1));
u_approx = (1/T)*int(1, t, 0, D*T);
for n = 1:order_approx
    u_approx = u_approx + (1/T)*int(exp(-1i*n*omega0*t), t, 0, D*T)*(exp(1i*n*omega0*t));
    u_approx = u_approx + (1/T)*int(exp(1i*n*omega0*t), t, 0, D*T)*(exp(-1i*n*omega0*t));
    basis(2*n) = cos(n*omega0*t);
    basis(2*n + 1) = sin(n*omega0*t);
end
u_approx = simplify(rewrite(u_approx,"sincos"));

% Preallocate symbolic array for phase-shifted signals
syms un un_approx [n_phases 1]

% Generate phase-shifted PWM signals
for i = 1:n_phases
    t_delay = (i - 1)*T/n_phases;
    un(i) = subs(u, t, t - t_delay);
    un_approx(i) = subs(u_approx, t, t - t_delay);
end

% Plot the original PWM signals
plota = true; % Bool para plot
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

syms In_L [n_phases 1]
syms LHS RHS [n_eq 1]
syms V_C Vin R_ind L C R VD real

for i = 1:n_eq
    if i <= n_phases
        In_L(i) = sum(xn((1:n_states_eq) + n_states_eq*(i - 1)).*basis);
        LHS(i) = diff(In_L(i), t);
    else
        V_C = sum(xn((1:n_states_eq) + n_states_eq*(i - 1)).*basis);
        LHS(i) = diff(V_C, t);
    end
end

RHS(end) = -V_C/(R*C);
for i = 1:n_phases
    RHS(i) = (1/L)*(Vin - R_ind*In_L(i) - V_C*(1 - un_approx(i)));
    RHS(end) = RHS(end) + In_L(i)*(1 - un_approx(i))/C;
end

syms Xn Xn_dot [n_states 1]

old_vars = [xn_dot; xn];
new_vars = [Xn_dot; Xn];

for i = 1:n_eq
    LHS(i) = subsExpression(LHS(i), old_vars, new_vars);
    LHS(i) = collect(LHS(i), basis);
    RHS(i) = subsExpression(RHS(i), old_vars, new_vars);
    RHS(i) = collect(RHS(i), basis);
end

syms coeffs_LHS coeffs_RHS [n_eq n_states_eq]

for i = 1:n_eq
    sep_coeffs_LHS = LHS(i);
    sep_coeffs_RHS = RHS(i);
    for n_basis = n_states_eq:-1:2
        sep_coeffs_LHS = coeffs(sep_coeffs_LHS, basis(n_basis));
        coeffs_LHS(i, n_basis) = sep_coeffs_LHS(2);
        sep_coeffs_LHS = sep_coeffs_LHS(1);
    
        sep_coeffs_RHS = coeffs(sep_coeffs_RHS, basis(n_basis));
        coeffs_RHS(i, n_basis) = sep_coeffs_RHS(2);
        sep_coeffs_RHS = sep_coeffs_RHS(1);
    end
    coeffs_LHS(i, 1) = sep_coeffs_LHS;
    coeffs_RHS(i, 1) = sep_coeffs_RHS;
end

eqs = coeffs_LHS(:) == coeffs_RHS(:);

% Solve for X_dot
solution = solve(eqs, Xn_dot);

% Convert struct to symbolic vector
X_dot = struct2array(solution);
X_dot = X_dot(:);

% Define state vector X and input u
X = Xn;
U = Vin; % Assume Vin is the input

% Express system in matrix form: X_dot = A*X + B*U
A = jacobian(X_dot, X); % Partial derivatives w.r.t. states
B = jacobian(X_dot, U); % Partial derivatives w.r.t. input

% Output y and C matrix
C_mat = kron(eye(n_eq), basis');

% Display results
% disp('State-space matrix A:');
% disp(A);
% disp('Input matrix B:');
% disp(B);
% disp('Observer matrix C:');
% disp(C_mat);

suffix = sprintf('_%d_order', order_approx); % Create the suffix dynamically

matlabFunction(A, 'File', ['function_A' suffix]);
matlabFunction(B, 'File', ['function_B' suffix]);
matlabFunction(C_mat, 'File', ['function_C' suffix]);