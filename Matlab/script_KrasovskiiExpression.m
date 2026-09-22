clear; close all; clc
n_phases = 3; % Number of phases
order_approx = 0; % Approximation order

dictionaries

save_expr_conv = true;
save_expr = false;

%% 
d_dot = piecewise(sym(1) == sym(1), 0);

K_u_pw = piecewise(sym(1) == sym(1), 0);
Kp_i_pw = piecewise(sym(1) == sym(1), 0);
Kp_v_pw = piecewise(sym(1) == sym(1), 0);

convs = ["Boost" "Buck"];
for conv = convs
    if conv == "Boost"
        sw_mode = 1;
    else
        sw_mode = 0;
    end
    X_dot = interleavedConverterModel(n_phases, order_approx, conv, sw_mode);
    recoverVarsExpr;

    syms vc(t) vc_dot(t)
    syms K_u positive
            
    syms il(t) il_dot(t) u(t) u_dot [n_phases 1]
    
    S = 0.5*L*sum(il_dot.^2) + 0.5*C*vc_dot^2;
    S_dot = diff(S);
    S_dot = subs(S_dot, diff(vc), vc_dot);

    X_dot_aux = subs(X_dot, [X; d], [il; vc; u]);
    
    S_dot = subs(S_dot, [diff(il_dot); diff(vc_dot)], diff(X_dot_aux));
    S_dot = subs(S_dot, diff([il; vc; u]), [il_dot; vc_dot; u_dot]);
    S_dot = simplify(S_dot);
    S_dot = collect(S_dot, u_dot);
    
    u_dot_sol = -K_u*jacobian(S_dot, u_dot).';

    %% 
    syms IL_dot IL [n_phases 1] real
    syms Vout_dot Vout real
    u_dot_sol = subs(u_dot_sol, il_dot, IL_dot);
    u_dot_sol = subs(u_dot_sol, vc_dot, Vout_dot);
    u_dot_sol = subs(u_dot_sol, il, IL);
    u_dot_sol = subs(u_dot_sol, vc, Vout);
    
    u_dot_sol = formula(u_dot_sol);

    syms Kp_v Kp_i positive
    syms IL_ref Vout_ref d_ref positive

    u_dot_sug = u_dot_sol - Kp_v*(Vout - Vout_ref) - Kp_i*(IL - IL_ref);
    X_aug_dot = [X_dot; u_dot_sug];

    X_aug_dot = subs(X_aug_dot, X, [IL; Vout]);
    X_aug_dot = subs(X_aug_dot, IL_dot, X_aug_dot(1:n_phases));
    X_aug_dot = subs(X_aug_dot, Vout_dot, X_aug_dot(n_phases + 1));
    X_aug_dot = simplify(expand(X_aug_dot), 1000);

    I_load_ref = solve(X_aug_dot(n_phases + 1), I_load);
    I_load_ref = subs(I_load_ref, IL, IL_ref*ones(n_phases, 1));
    I_load_ref = subs(I_load_ref, d, d_ref*ones(n_phases, 1));
    Vin_ref = solve(X_aug_dot(1), Vin);
    Vin_ref = subs(Vin_ref, IL, IL_ref*ones(n_phases, 1));
    Vin_ref = subs(Vin_ref, d, d_ref*ones(n_phases, 1));

    jac_X_aug_dot = jacobian(X_aug_dot, [IL; Vout; d]);
    jac_X_aug_dot = simplify(expand(jac_X_aug_dot), 1000);

    d_eq = solve(X_aug_dot(1:n_phases) == 0, d);
    if isstruct(d_eq)
        d_eq = simplify(struct2array(d_eq)');
    end

    X_aug_dot_eq = simplify(subs(X_aug_dot, d, d_eq));

    expr_Vout_dot = subs(X_aug_dot_eq(n_phases + 1), IL, IL_ref*ones(n_phases, 1));
    IL_eq = solve(expr_Vout_dot == 0, IL_ref);
    IL_eq = IL_eq(~isnan(limit(IL_eq, R_L, 0)));
    X_aug_dot_eq = simplify(subs(X_aug_dot_eq, IL, IL_eq*ones(n_phases, 1)));

    jac_X_aug_dot_ref = jac_X_aug_dot;
    if any(has(jac_X_aug_dot_ref, I_load), 'All')
        jac_X_aug_dot_ref(has(jac_X_aug_dot, I_load)) = ...
            subs(jac_X_aug_dot_ref(has(jac_X_aug_dot, I_load)), IL, IL_ref*ones(n_phases, 1));
        jac_X_aug_dot_ref(has(jac_X_aug_dot, I_load)) = ...
            subs(jac_X_aug_dot_ref(has(jac_X_aug_dot, I_load)), d, d_ref*ones(n_phases, 1));
        jac_X_aug_dot_ref(has(jac_X_aug_dot, I_load)) = ...
            subs(jac_X_aug_dot_ref(has(jac_X_aug_dot, I_load)), I_load, I_load_ref);
    end
    if any(has(jac_X_aug_dot_ref, Vin), 'All')
        jac_X_aug_dot_ref(has(jac_X_aug_dot_ref, Vin)) = ...
            subs(jac_X_aug_dot_ref(has(jac_X_aug_dot_ref, Vin)), d, d_eq);
        jac_X_aug_dot_ref = simplify(jac_X_aug_dot_ref);
        jac_X_aug_dot_ref = collect(jac_X_aug_dot_ref, K_u);
    
        if sw_mode == 1
            [d_bar_num, d_bar_den] = numden(d_eq);
            jac_X_aug_dot_ref = subs(jac_X_aug_dot_ref, d_bar_num, d_ref*d_bar_den);
        else
            [d_bar_num, d_bar_den] = numden(1 - d_eq);
            jac_X_aug_dot_ref = subs(jac_X_aug_dot_ref, d_bar_num, (1 - d_ref)*d_bar_den);
        end       
    end
    
    jac_X_aug_dot_ref = subs(jac_X_aug_dot_ref, [IL; Vout; d], ...
        [IL_ref*ones(n_phases, 1); Vout_ref; d_ref*ones(n_phases, 1)]);

    jac_X_aug_dot_ref = subs(jac_X_aug_dot_ref, VD, 0);

    n_states = n_phases*2 + 1;

    jac_X_aug_dot_ref_scalar = simplify(jac_X_aug_dot_ref);
    eig_values = eig(jac_X_aug_dot_ref_scalar);

    [first_order, second_order, third_order] = classify_eigenvalues(eig_values);

    [sol] = paramGains(first_order, second_order, third_order);

    syms zeta ts k positive
    assumeAlso(k < 0.5) % condition for zeta^2 to be positive

    eig_2nd = second_order{1}.eigenvalues;
    eig_3rd = third_order{1}.eigenvalues;

    %
    L_val = 80e-3;
    C_val = 12e-3;
    R_L_val = 1e-3;
    Vin_val = 12;
    IL_ref_val = 1;
    Vout_ref_val = 24;
    zeta_val = sqrt(2)/2;
    ts_val = 0.1;
    k_val = 0.5;
    
    if conv == "Boost"
        d_ref_val = 1 - (Vin_val - R_L_val*IL_ref_val)/Vout_ref_val;
    else
        d_ref_val = Vout_ref_val/Vin_val;
    end

    vars_orig = [L C R_L Vin IL_ref Vout_ref d_ref zeta ts k];
    vars_new = [L_val C_val R_L_val Vin_val IL_ref_val Vout_ref_val d_ref_val zeta_val ts_val k_val];

    n_sols = length(sol.K_u);
    valid_idx = true(n_sols, 1);

    for i = 1:n_sols
        K_u_i   = sol.K_u(i);
        Kp_i_i  = sol.Kp_i(i);
        Kp_v_i  = sol.Kp_v(i);
        
        % eig_2nd_sol = subs(eig_2nd, [K_u Kp_i Kp_v], [K_u_i Kp_i_i Kp_v_i]);
        eig_3rd_sol = subs(eig_3rd, [K_u Kp_i Kp_v], [K_u_i Kp_i_i Kp_v_i]);
    
        % eig_2nd_sol = simplify(eig_2nd_sol);
        eig_3rd_sol = simplify(eig_3rd_sol);
        
        K_u_val = double(subs(K_u_i, vars_orig, vars_new));
        Kp_i_val = double(subs(Kp_i_i, vars_orig, vars_new));
        Kp_v_val = double(subs(Kp_v_i, vars_orig, vars_new));
        
        vars_orig = [vars_orig K_u Kp_i Kp_v]; %#ok<AGROW>
        vars_new = [vars_new K_u_val Kp_i_val Kp_v_val]; %#ok<AGROW>
        
        % eig_2nd_val = double(subs(eig_2nd_sol, [vars_orig K_u Kp_i Kp_v], [vars_new K_u_val Kp_i_val Kp_v_val]));
        eig_3rd_val = double(subs(eig_3rd_sol, [vars_orig K_u Kp_i Kp_v], [vars_new K_u_val Kp_i_val Kp_v_val]));
        
        % resultsTable_2nd = eigenvalueMetrics(eig_2nd_val);
        resultsTable_3rd = eigenvalueMetrics(eig_3rd_val);

        % disp(resultsTable_2nd);
        disp(resultsTable_3rd);

        ts_3rd = resultsTable_3rd.SettlingTime_Ts;
        zeta_3rd = resultsTable_3rd.Zeta;

        ts_expected = sort([k_val*ts_val; ts_val; ts_val]);
        ts_check = all((sort(ts_3rd) - ts_expected).^2 < 1e-6);

        zeta_expected = sort([1; zeta_val; zeta_val]);
        zeta_check = all((sort(zeta_3rd) - zeta_expected).^2 < 1e-6);

        valid_idx(i) = ts_check && zeta_check;
    end

    K_u_conv = sol.K_u(valid_idx);
    Kp_v_conv = sol.Kp_v(valid_idx);
    Kp_i_conv = sol.Kp_i(valid_idx);

    syms converter
    K_u_pw = piecewise(converter == map_conv(conv), K_u_conv, K_u_pw);
    Kp_v_pw = piecewise(converter == map_conv(conv), Kp_v_conv, Kp_v_pw);
    Kp_i_pw = piecewise(converter == map_conv(conv), Kp_i_conv, Kp_i_pw);

    [~, u_dot_sug] = equivalentExpressions(u_dot_sug);
    d_dot = piecewise(converter == map_conv(conv), u_dot_sug, d_dot);

    if save_expr_conv
        file_name = "Krasovskii_duty_" + conv;
        matlabFunction(u_dot_sug, ...
             'Outputs', {'d_dot'}, ...
             'File', file_name, 'Optimize', false); %#ok<*UNRCH>
        
        gains = [K_u_conv; Kp_v_conv; Kp_i_conv];
        file_name = "Krasovskii_gains_" + conv;
        matlabFunction(gains, ...
             'Outputs', {'gains_expr'}, ...
             'File', file_name, 'Optimize', false);
    end
end

%% Salva as expressoes (copie e cole)
if save_expr
    matlabFunction(d_dot, ...
         'Outputs', {'d_dot'}, ...
         'File', "Krasovskii_duty", 'Optimize', false); %#ok<*UNRCH>
    
    gains = [K_u_pw; Kp_v_pw; Kp_i_pw];
    matlabFunction(gains, ...
         'Outputs', {'gains_expr'}, ...
         'File', "Krasovskii_gains", 'Optimize', false);
end