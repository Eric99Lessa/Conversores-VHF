clear

n_phases = 3; % Number of phases
n_states_eq = 1;
n_eq = n_phases + 1; % 1 equation for capacitor and 1 for each inductor
n_states = n_eq;

%% States variables
syms D real
syms xn xn_dot [n_states 1]

for i = 1:n_states
    syms(sprintf('x%d(t)', i)) %declare each element in the array as a single symbolic function
    xn(i) = symfun(eval(sprintf('x%d(t)', i)), t); %declare each element to a symbolic "handle"
    xn_dot(i) = diff(xn(i), t);
end

syms In_L [n_phases 1]
syms LHS RHS [n_eq 1]
syms V_C Vin I_load L C VD real

for i = 1:n_eq
    if i <= n_phases
        In_L(i) = sum(xn((1:n_states_eq) + n_states_eq*(i - 1)));
        LHS(i) = diff(In_L(i), t);
    else
        V_C = sum(xn((1:n_states_eq) + n_states_eq*(i - 1)));
        LHS(i) = diff(V_C, t);
    end
end

RHS(end) = -I_load/C;
for i = 1:n_phases
    RHS(i) = (Vin - V_C*(1 - D))/L;
    RHS(end) = RHS(end) + In_L(i)*(1 - D)/C;
end

syms Xn Xn_dot [n_states 1]

old_vars = [xn_dot; xn];
new_vars = [Xn_dot; Xn];

for i = 1:n_eq
    LHS(i) = subsExpression(LHS(i), old_vars, new_vars);
    RHS(i) = subsExpression(RHS(i), old_vars, new_vars);
end

syms coeffs_LHS coeffs_RHS [n_eq n_states_eq]

for i = 1:n_eq
    sep_coeffs_LHS = LHS(i);
    sep_coeffs_RHS = RHS(i);
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
U = D; % Assume Vin is the input

% Express system in matrix form: X_dot = A*X + B*U
A = jacobian(X_dot, X); % Partial derivatives w.r.t. states
B = jacobian(X_dot, U); % Partial derivatives w.r.t. input

% Output y and C matrix
C_mat = eye(n_eq);