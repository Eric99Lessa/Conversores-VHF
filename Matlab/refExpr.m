function [IL_ref, d_ref] = refExpr(X_dot)
    %%
    [~, ~, ~, g_x, ~] = DE2pH(X_dot);

    recoverVarsExpr;

    %% 
    n = length(X) - 1;

    %% Acha o duty cycle de acordo com o modo escolhido   
    syms Vout IL_eq n_phases real
    syms IL [n 1] real

    null_g_x = null(g_x')';

    res = null_g_x*X_dot;
    res = subs(res, X, [IL; Vout]);
    res = subs(res, IL, IL_eq*ones(size(IL)));
    
    % normally there will be two solutions since there are terms R*IL^2
    % but only one will be stable for R -> 0 and Vin > 0

    IL_sol = solve(res == 0, IL_eq);
    if length(IL_sol) > 1
        assumeAlso(Vin > 0);
        IL_ref_check = ~isnan(limit(IL_sol, R_L, 0));
        IL_sol = IL_sol(IL_ref_check);
    end

    IL_ref = IL_sol;

    %%
    d_ref = solve(X_dot(1:n) == 0, d);
    if isstruct(d_ref)
        d_ref = struct2array(d_ref);
        d_ref = d_ref(:);
    end
    d_ref = subs(d_ref, X, [IL; Vout]);
    d_ref = subs(d_ref, IL, IL_ref*ones(n, 1));
    d_ref = simplify(d_ref);

    [~, d_ref] = equivalentExpressions(d_ref);
    d_ref = simplify(expand(d_ref), 1000);
end