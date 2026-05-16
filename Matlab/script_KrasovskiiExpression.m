clear; close all;
n_phases = 3; % Number of phases
order_approx = 0; % Approximation order

save_expr = true;

dictionaries

%% 
d_dot = piecewise(sym(1) == sym(1), 0);

convs = ["Boost" "Buck" "BuckBoost"];
for conv = convs
    X_dot = interleavedConverterModel(n_phases, order_approx, conv);
    
    recoverVarsExpr;
    
    syms vc(t) vc_dot(t) vc_ref d_ref K_u
            
    syms il(t) il_dot(t) u(t) u_dot [n_phases 1]
    
    S = 0.5*L*sum(il_dot.^2) + 0.5*C*vc_dot^2;
    S_dot = diff(S);
    
    X_dot_aux = subs(X_dot, [X; d], [il; vc; u]);
    
    S_dot = subs(S_dot, [diff(il_dot); diff(vc_dot)], diff(X_dot_aux));
    S_dot = subs(S_dot, diff([il; vc; u]), [il_dot; vc_dot; u_dot]);
    S_dot = subs(S_dot, VD, 0);
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

    syms converter
    [~, u_dot_sol] = equivalentExpressions(u_dot_sol);
    d_dot = piecewise(converter == map_conv(conv), u_dot_sol, d_dot);
end

if save_expr
    matlabFunction(d_dot, 'File', "d_dot_expr", 'Optimize', false);
end