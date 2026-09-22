function [d_num, d_den, Kj_sol, Kr_sol, Ki_sol] = controlLawIDAPBCExpr(X_dot, integrator)
    arguments
        X_dot;
        integrator = 0;
    end

    %% Recover symbolic variables used in X_dot
    recoverVarsExpr;
    n = length(X) - 1; %#ok<NODEF> % number of phases

    %% Get current reference
    % [IL_ref, ~] = refExpr(X_dot);

    %% Add integrator
    if integrator
        syms Ki Vout_ref positive
        X_dot = [X_dot; X(n + 1) - Vout_ref];
    end

    %% Get the pH matrixes
    [J, R, jac_X, g_x, xi] = DE2pH(X_dot);

    %%
    syms Kr Kj positive
    syms e_Vout int_eVout real
    syms e_IL [n, 1] real
    assume(Kr > 1); assume(Kj > 1);

    err = [e_IL; e_Vout];
    if integrator
        X = [X; int_eVout];
        err = [err; int_eVout];
    end

    %%
    % Define closed-loop Hamiltonian
    w = sqrt(diag(jac_X));
    Hd = 0.5*(w.*err).'*(w.*err);
    jac_Hd = jacobian(Hd, err)';
    jac_Xd = jacobian(jac_Hd, err);

    Jd = J;
    Jd(1:n+1, 1:n+1) = Kj*Jd(1:n+1, 1:n+1);
    Rd = Kr*R;

    jac_d = (Jd - Rd)*jac_Xd;
    if integrator
        jac_d(1:n, end) = -Ki*jac_d(end, n + 1);
    end
    eig_values = eig(jac_d);
    [first_order, second_order, third_order] = classify_eigenvalues(eig_values);

    [sol] = paramGains(first_order, second_order, third_order);

    Kj_sol = sol.Kj;
    Kr_sol = sol.Kr;

    if integrator
        Ki_sol = sol.Ki;

        idx_Ki_pos = isAlways(Ki_sol > 0);
        Kj_sol = Kj_sol(idx_Ki_pos);
        Kr_sol = Kr_sol(idx_Ki_pos);
        Ki_sol = Ki_sol(idx_Ki_pos);
    else
        Ki_sol = [];
    end

    %% Lei de Controle
    
    % Equacao em malha aberta: X_dot = (J - R)*jac_H + g_x*U + xi
    % Equacao desejada em malha fechada: X_dot = err_dot = (Jd - Rd)*jac_Hd
    % Igualando as duas expressoes:
    % (J - R)*jac_H + g_x*u + xi = (Jd - Rd)*jac_Hd
    % g_x*u = (Jd - Rd)*jac_Hd - (J - R)*jac_H - xi
    % u = pinv(g_x)*g_x*((Jd - Rd)*jac_Hd - (J - R)*jac_H - xi) (1)

    % Se a parte nao controlavel obedecer a null(g_x')'*X_dot == 0
    % A expressao (1) garante dinamica igual a desejada

    %% Acha o duty cycle
    pinv_g_x = simplify(collect(pinv(g_x)));

    expr = jac_d*X - (J - R)*jac_X*X - xi;
    d_sol = pinv_g_x*expr;
    d_sol = simplify(d_sol);

    syms IL [n 1]
    syms Vout
    d_sol = subs(d_sol, X(1:n+1), [IL; Vout]);
    [d_num, d_den] = numden(d_sol);
    
    d_num = simplify(d_num);
    d_den = simplify(d_den);
end