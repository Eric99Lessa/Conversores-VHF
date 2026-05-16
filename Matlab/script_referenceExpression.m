clear; clc; close all;

order_approx = 0; % Approximation order

save_expr = true;

%% 

dictionaries

syms converter n_phases

IL_ref = piecewise(sym(1) == sym(1), 0);
d_ref = piecewise(sym(1) == sym(1), 0);

ns_phases = 1:3;
convs = ["Buck" "Boost" "BuckBoost"];

for conv = convs
    for n = ns_phases
        [X_dot] = interleavedConverterModel(n, order_approx, conv);
        
        [IL_ref_conv, d_ref_conv] = refExpr(X_dot);

        IL_ref = piecewise(n_phases == n & converter == map_conv(conv), IL_ref_conv, IL_ref);
        d_ref = piecewise(n_phases == n & converter == map_conv(conv), d_ref_conv, d_ref);
    end
end
if save_expr
    matlabFunction(IL_ref, 'File', "IL_ref_expr", 'Optimize', false);
    matlabFunction(d_ref, 'File', "d_ref_expr", 'Optimize', false);
end

