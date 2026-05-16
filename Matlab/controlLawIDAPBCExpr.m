function [d_num, d_den, jac_d, jac_mf, res_mf, eig_values] = controlLawIDAPBCExpr(X_dot, mode_solv)
    arguments
        X_dot,
        mode_solv = 0,
    end

    %%
    [J, R, jac_X, g_x, xi] = DE2pH(X_dot);

    recoverVarsExpr;
    %% 
    n_phases = length(X) - 1;

    %%
    syms e_Vout IL_ref_dot Kr Kj real
    syms e_IL [n_phases 1] real
    assumeAlso(Kr > 0);
    assumeAlso(Kj > 0);

    X_ref_dot = [IL_ref_dot*ones(n_phases, 1); 0];
    err = [e_IL; e_Vout];

    %%
    J_aux = J;
    R_aux = R;
    
    Jc = Kj*J_aux;
    Rc = Kr*R_aux;

    %% 
    
    % Define closed-loop Hamiltonian
    w = sqrt(diag(jac_X));
    Hd = 0.5*(w.*err).'*(w.*err);
    jac_Hd = jacobian(Hd, err)';
    jac_Xd = jacobian(jac_Hd, err);
    
    Jd = simplify(J_aux + Jc);
    Jd = subs(Jd, Kj + 1, Kj); assume(Kj > 1);
    Rd = simplify(R_aux + Rc);
    Rd = subs(Rd, Kr + 1, Kr); assume(Kr > 1);

    if nargout > 2
        jac_d = (Jd - Rd)*jac_Xd;
    end

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

    syms Vout IL_ref real
    syms IL [n_phases 1] real

    g_x = subs(g_x, X, [IL; Vout]);
    
    expr_ma = (J - R)*jac_X*[IL; Vout] + g_x*d + xi; % X_dot malha aberta
    expr_mf_d = (Jd - Rd)*jac_Xd*err + X_ref_dot; % X_dot malha fechada
    
    %%
    g_perp = null(g_x.')';
    
    res = g_perp*expr_ma;
    res = simplify(res);
    [num_res, ~] = numden(res);

    num_res = subs(num_res, IL, IL_ref);
    
    % normally there will be two solutions since there are terms R*IL^2
    % but only one will be stable for R -> 0 and Vin > 0

    IL_sol = solve(num_res == 0, IL_ref);
    if length(IL_sol) > 1
        assumeAlso(Vin > 0);
        IL_ref_check = ~isnan(limit(IL_sol, R_L, 0));
        IL_sol = IL_sol(IL_ref_check);
    end

    % num_res = sum(num_res)/2 -
    % Isolando g_x*d
    LHS = g_x*d;
    RHS = simplify(expr_mf_d(1:end-1) - expr_ma + LHS);

    if exist("R_C", 'var')
        RHS = simplify(subs(expand(RHS), R_C, inf));
    end

    %% Acha o duty cycle de acordo com o modo escolhido   
    if mode_solv == 0 % current equations IDA-PBC
        idx_eq = 1:n_phases;
        d_sol = solve(LHS(idx_eq) == RHS(idx_eq), d);

        if isstruct(d_sol)
            d_sol = simplify(struct2array(d_sol)');
        end
    elseif mode_solv == 1 % PINV solution IDA-PBC
        pinv_g_x = simplify(collect(pinv(g_x)));
        d_sol = pinv_g_x*RHS;  
    end

    d_sol = subs(d_sol, X, [IL; Vout]);
    [d_num, d_den] = numden(d_sol);

    if all(d_den == d_den(1))
        d_den = d_den(1);
    end
    
    d_num = simplify(d_num);
    d_den = simplify(d_den);

    %%
    if nargout > 3
        syms IL_ref Vout_ref real

        err_expr = [e_IL; e_Vout];
        err = [IL - IL_ref; Vout - Vout_ref];

        expr_mf = (J - R)*jac_X*X + g_x*(d_num./d_den) + xi;
        expr_mf = subs(expr_mf, X, [IL; Vout]);
        expr_mf = subs(expr_mf, err_expr, err);
        expr_mf = simplify(expand(expr_mf), 1000);
        % expr_mf = simplify(subs(expr_mf, IL_ref, IL_sol), 1000);
        
        jac_mf = jacobian(expr_mf, [IL; Vout]);
        jac_mf = simplify(jac_mf, 1000);
        jac_mf = subs(jac_mf, IL, IL_ref);
        jac_mf = subs(jac_mf, Vout, Vout_ref);
        jac_mf = subs(jac_mf, IL_ref_dot, 0);
        
        if nargout > 4
            res_mf = simplify(expand(expr_mf - X_ref_dot - jac_mf*err), 1000);
            res_mf = res_mf(res_mf ~= 0);
            if exist("R_C", 'var')
                res_mf = simplify(subs(expand(res_mf), R_C, inf));
            end
            res_mf = simplify(res_mf, 1000);
        end
        if nargout > 5
            eig_values = eig(jac_mf);
            if ~isempty(regexp(string(eig_values(1)), 'root', 'once'))
                str_roots = string(eig_values(1));
                [~, noMatch] = regexp(str_roots, 'root\(.*?', 'match', 'split');
                noMatch = noMatch(noMatch ~= "");
                poly_expr = strsplit(noMatch, ',');
                poly_expr = poly_expr(1);
                poly_expr = str2sym(poly_expr);
                poly_expr = simplify(poly_expr);
                eig_values = solve(poly_expr);
            end
            eig_values = simplify(eig_values, 1000);
        end
    end
end