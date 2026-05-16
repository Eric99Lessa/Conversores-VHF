clear; clc; close all;

n_phases = 3; % Number of phases
order_approx = 0; % Approximation order

save_expr = true;

dictionaries;

%% 

duty = piecewise(sym(1) == sym(1), 0);
Kp_v = piecewise(sym(1) == sym(1), 0);
Ki_v = piecewise(sym(1) == sym(1), 0);

convs = ["Boost" "Buck" "BuckBoost"];
for conv = convs
    X_dot = interleavedConverterModel(n_phases, order_approx, conv);
    
    [duty_conv, Kp_i_conv, Kp_v_conv, Ki_v_conv, tau_sol] = controlLawCascadeExpr(X_dot, conv);

    syms converter

    [~, duty_conv] = equivalentExpressions(duty_conv);

    duty = piecewise(converter == map_conv(conv), duty_conv, duty);
    Kp_v = piecewise(converter == map_conv(conv), Kp_v_conv, Kp_v);
    Ki_v = piecewise(converter == map_conv(conv), Ki_v_conv, Ki_v);
end

if save_expr
    matlabFunction(duty, 'File', "duty_cascade_expr", 'Optimize', false);
    matlabFunction([Kp_v Ki_v], 'File', "gains_cascade_expr", 'Optimize', false, 'Outputs', {'Kp_Ki'});
end