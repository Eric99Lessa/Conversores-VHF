clear; clc; close all;

n_phases = 3; % Number of phases
order_approx = 0; % Approximation order

save_expr = true;
save_gain = false;

dictionaries;

%% 

alpha_IL = piecewise(sym(1) == sym(1), 0);

convs = ["Boost" "Buck" "BuckBoost"];
for conv = convs
    X_dot = interleavedConverterModel(n_phases, order_approx, conv);
    
    [~, ~, ~, g_x, ~] = DE2pH(X_dot);

    recoverVarsExpr;

    syms Vout real
    syms IL [n_phases 1]
    g_x = subs(g_x, X, [IL; Vout]);

    alpha_IL_conv = diag(g_x(1:n_phases, :));

    %%
    [~, alpha_IL_conv] = equivalentExpressions(alpha_IL_conv);

    syms converter

    alpha_IL = piecewise(converter == map_conv(conv), alpha_IL_conv, alpha_IL);
end

if save_expr
     matlabFunction(alpha_IL, 'File', "alpha_modelFree_expr", 'Optimize', false);
end