function [IL_ref, d_ref] = refExpr(X_dot)
    %% Recover symbolic variables used in X_dot
    recoverVarsExpr;
    n = length(X) - 1; % number of phases

    %% Get the pH matrixes
    [~, ~, ~, g_x, ~] = DE2pH(X_dot);

    %% Find non controllable part of X_dot
    syms IL_eq Vout

    null_g_x = null(g_x')';
    res = null_g_x*X_dot;
    res = subs(res, X, [IL_eq*ones(n, 1); Vout]);
    res = simplify(res);
    [num_res, ~] = numden(res);

    %% Uses non controllable part to get current reference
    IL_ref = solve(num_res == 0, IL_eq);
    if length(IL_ref) > 1
        assumeAlso(Vin > 0);
        IL_ref_check = ~isnan(limit(IL_ref, R_L, 0));
        IL_ref = IL_ref(IL_ref_check);
    end

    if nargout > 1
        %% Finds reference duty cycle in IL_ref and Vout_ref
        syms Vout
    
        d_ref = solve(X_dot(1:n) == 0, d);
        if isstruct(d_ref)
            d_ref = struct2array(d_ref);
            d_ref = d_ref(:);
        end
        d_ref = subs(d_ref, X, [IL_ref*ones(n, 1); Vout]);
        d_ref = simplify(d_ref);
    
        [~, d_ref] = equivalentExpressions(d_ref);
        d_ref = simplify(expand(d_ref), 1000);
    end
end