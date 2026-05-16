function [J, R, jac_X, g, xi] = DE2pH(X_dot)
    recoverVarsExpr;

    %% 
    U = d;
    
    integrator = exist('Ki', 'var');
    n_phases = length(X) - 1 - integrator;

    %% Define Hamiltonian and jacobian
    if integrator
        mult_X = sqrt([L*ones(n_phases, 1); C; 1/Ki]);
    else
        mult_X = sqrt([L*ones(n_phases, 1); C]);
    end
    Q = diag(mult_X);

    H = 0.5*(Q*X).'*(Q*X);
    jac_H = jacobian(H, X); jac_H = jac_H(:);
    jac_X = jacobian(jac_H, X);
    
    %% Find each desired matrix
    % Input Matrix
    g = jacobian(X_dot, U);
    
    X_dot_aux = simplify(X_dot - g*U);
    
    % External
    xi = subs(X_dot_aux, X, zeros(size(X)));
    
    X_dot_aux = simplify(X_dot_aux - xi);
    
    J_minus_R = jacobian(X_dot_aux, X);
    J_minus_R = J_minus_R/jac_X;
    
    R = -diag(diag(J_minus_R));
    
    J = J_minus_R + R;
    % R = -(J_minus_R + J_minus_R')/2;
    % J = (J_minus_R - J_minus_R')/2;
    
    % X_dot reescrito
    % test = X_dot - ((J - R)*jac_X*X + g*U + xi);
    % pH_form_check = all(isAlways(simplify(test) == 0));
end