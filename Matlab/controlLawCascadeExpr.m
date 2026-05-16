function [d_expr, Kp_i_sol, Kp_v_sol, Ki_v_sol, tau_sol] = controlLawCascadeExpr(X_dot, converter)
    %%
    [~, ~, ~, g_x, ~] = DE2pH(X_dot);

    recoverVarsExpr;
    dictionaries;
     
    n_phases = length(X) - 1;

    %%
    syms Vout Vout_ref IL_ref real
    syms IL [n_phases 1] real

    %% Lei de Controle

    syms Kp_i Kp_v Ki_v positive
    syms IL_eq_ref int_eVout real
    assume(Kp_v > 0)
    assume(Ki_v > 0)
    X_dot = subs(X_dot, X, [IL; Vout]);
    X_dot = subs(X_dot, VD, 0);
    
    % No controle em cascata vamos cancelar totalmente a dinamica do
    % conversor usando as equações de corrente e impor a referencia de
    % tensão usando a corrente de referencia 

    d_sol = solve(X_dot(1:n_phases) == 0, d);

    if isstruct(d_sol)
        d_sol = simplify(struct2array(d_sol)');
    end
    [d_num_sol, d_den_sol] = numden(d_sol);
    
    d_expr = (d_num_sol - Kp_i*(IL - IL_ref))./d_den_sol;

    %%
    syms u_IL_eq
    IL_eq = IL_ref_expr(I_load,R_L,VD,Vin,Vout,map_conv(converter),n_phases);
    IL_eq = subs(IL_eq, VD, 0);
    IL_eq = subs(IL_eq, Vout, Vout_ref);
    
    u_IL = -(Kp_v*(Vout - Vout_ref) + Ki_v*int_eVout)/n_phases;

    d_expr = subs(d_expr, IL_ref, IL_eq_ref + u_IL);
    d_expr = simplify(d_expr, 1000);

    %%
    IL_dot = X_dot(1:n_phases);
    IL_dot = subs(IL_dot, X, [IL; Vout]);
    IL_dot = subs(IL_dot, d, d_expr);
    IL_dot = simplify(IL_dot, 1000);

    %%
    Vout_dot = X_dot(end);
    Vout_dot = subs(Vout_dot, X, [IL; Vout]);
    Vout_dot = subs(Vout_dot, d, d_expr);
    Vout_dot = simplify(expand(Vout_dot));

    %%
    int_eVout_dot = Vout - Vout_ref;
    jac_Xdot = jacobian([IL_dot; Vout_dot; int_eVout_dot], [IL; Vout; int_eVout]);
    jac_err = subs(jac_Xdot, IL, IL_eq_ref*ones(n_phases, 1));
    jac_err = subs(jac_err, Vout, Vout_ref);
    jac_err = subs(jac_err, int_eVout, 0);

    jac_err = simplifyFraction(jac_err);
    jac_err = limit(jac_err, R_L, 0);

    %%
    syms s
    poly_s = collect(det(eye(size(jac_err, 1))*s - jac_err), s);

    % In all interleaved converters (buck, boost and buck-boost) -Kp_i is
    % an eigen value (since we impose this in the current loop controller
    % so we can choose the Kp_i value based on desired settling time Ts_i

    mult_Kp_i = 0;
    r = sym(0);
    while isequal(r, sym(0))
        [r, q] = polynomialReduce(poly_s, s + Kp_i, s);

        r = simplify(r);
        if isequal(r, sym(0))
            mult_Kp_i = mult_Kp_i + 1;
            poly_s = simplify(collect(q, s));
        end
    end

    order_rem = size(jac_err, 1) - mult_Kp_i;
    coeffs_poly = coeffs(poly_s, s, 'All');
    coeffs_poly = simplify(expand(coeffs_poly));
    syms Ts_i wn_v zeta_v positive

    Kp_i_sol = 4/Ts_i; % 2% criteria

    if order_rem == 2
        eqs = coeffs_poly(2:end)' == [2*zeta_v*wn_v; wn_v^2];
        sol = solve(eqs, [Kp_v, Ki_v]);

        Kp_v_sol = sol.Kp_v(1);
        Ki_v_sol = sol.Ki_v(1);
    elseif order_rem == 3
        syms pole positive

        eqs = coeffs_poly(2:end)' == [2*zeta_v*wn_v + pole;
                                        wn_v^2 + 2*zeta_v*wn_v*pole;
                                        pole*wn_v^2];
        sol = solve(eqs, [pole, Kp_v, Ki_v]);

        tau_sol = simplify(1/sol.pole(1), 1000);
        Kp_v_sol = simplify(sol.Kp_v(1), 1000);
        Ki_v_sol = simplify(sol.Ki_v(1), 1000);
    end

    if map_conv(converter) == 0
        tau_sol = subs(tau_sol, Vout_ref, Vout_ref + VD);
        Kp_v_sol = subs(Kp_v_sol, Vout_ref, Vout_ref + VD);
        Ki_v_sol = subs(Ki_v_sol, Vout_ref, Vout_ref + VD);
    elseif map_conv(converter) == 1
        tau_sol = subs(tau_sol, Vin, Vin + VD);
        Kp_v_sol = subs(Kp_v_sol, Vin, Vin + VD);
        Ki_v_sol = subs(Ki_v_sol, Vin, Vin + VD);
    elseif map_conv(converter) == 2
        tau_sol = subs(tau_sol, Vin, Vin + VD);
        Kp_v_sol = subs(Kp_v_sol, Vin, Vin + VD);
        Ki_v_sol = subs(Ki_v_sol, Vin, Vin + VD);
    end
end