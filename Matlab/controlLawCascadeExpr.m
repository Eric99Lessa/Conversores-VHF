function [d_expr, Kp_i_sol, Kp_v_sol, Ki_v_sol] = controlLawCascadeExpr(X_dot)
    %%
    IL_eq = refExpr(X_dot);

    %% Recover symbolic variables used in X_dot
    recoverVarsExpr;
    n = length(X) - 1; % number of phases

    %%
    syms Vout Vout_ref IL_ref real
    syms IL [n 1] real

    %% Lei de Controle
    syms Kp_i Kp_v Ki_v positive
    syms IL_eq_ref int_eVout real
    X_dot = subs(X_dot, X, [IL; Vout]);
    
    % No controle em cascata vamos cancelar totalmente a dinamica do
    % conversor usando as equações de corrente e impor a referencia de
    % tensão usando a corrente de referencia 

    d_sol = solve(X_dot(1:n) == 0, d);

    if isstruct(d_sol)
        d_sol = simplify(struct2array(d_sol)');
    end
    [d_num_sol, d_den_sol] = numden(d_sol);
    
    d_expr = (d_num_sol - Kp_i*L*(IL - IL_ref))./d_den_sol;

    %%
    u_IL = Kp_v*((Vout - Vout_ref) + Ki_v*int_eVout);

    d_expr = subs(d_expr, IL_ref, IL_eq_ref - u_IL);
    d_expr = simplify(d_expr, 1000);

    %%
    int_eVout_dot = Vout - Vout_ref;
    syst = [X_dot; int_eVout_dot];
    syst = subs(syst, X, [IL; Vout]);
    syst = subs(syst, d, d_expr);
    syst = simplify(syst, 1000);

    %%
    jac_Xdot = jacobian(syst, [IL; Vout; int_eVout]);
    X_eq = [IL_eq_ref*ones(n, 1); Vout_ref; 0];
    jac_err = subs(jac_Xdot, [IL; Vout; int_eVout], X_eq);

    jac_err = simplifyFraction(jac_err);
    jac_err = limit(jac_err, R_L, 0);

    %%
    eig_values = eig(jac_err);
    eig_values = simplify(eig_values);
    [first_order, second_order, third_order] = classify_eigenvalues(eig_values);

    [sol] = paramGains(first_order, second_order, third_order);

    Ki_v_sol = sol.Ki_v;
    Kp_i_sol = sol.Kp_i;
    Kp_v_sol = sol.Kp_v;
end