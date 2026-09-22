clear; clc; close all;

n_phases = 3; % Number of phases
order_approx = 0; % Approximation order

save_expr_conv = true;
save_expr = false;

dictionaries;

%% 

duty = piecewise(sym(1) == sym(1), 0);
Kp_i = piecewise(sym(1) == sym(1), 0);
Kp_v = piecewise(sym(1) == sym(1), 0);
Ki_v = piecewise(sym(1) == sym(1), 0);

convs = ["Buck" "Boost" "Buck"];
for conv = convs
    X_dot = interleavedConverterModel(n_phases, order_approx, conv);
    
    [duty_conv, Kp_i_conv, Kp_v_conv, Ki_v_conv] = controlLawCascadeExpr(X_dot);

    syms converter

    [~, duty_conv] = equivalentExpressions(duty_conv);

    duty = piecewise(converter == map_conv(conv), duty_conv, duty);
    Kp_i = piecewise(converter == map_conv(conv), Kp_i_conv, Kp_i);
    Kp_v = piecewise(converter == map_conv(conv), Kp_v_conv, Kp_v);
    Ki_v = piecewise(converter == map_conv(conv), Ki_v_conv, Ki_v);

    if save_expr_conv
        file_name = "Cascade_duty_" + conv;
        matlabFunction(duty_conv, ...
         'Outputs', {'duty'}, ...
         'File', file_name, 'Optimize', false);
    
        gains = [Kp_i_conv; Kp_v_conv; Ki_v_conv];
        file_name = "Cascade_gains_" + conv;
        matlabFunction(gains, ...
             'Outputs', {'gains_expr'}, ...
             'File', file_name, 'Optimize', false);
    end
end

%% Salva as expressoes (copie e cole)
if save_expr
    matlabFunction(duty, ...
         'Outputs', {'duty'}, ...
         'File', "Cascade_duty", 'Optimize', false); %#ok<*UNRCH>
    
    gains = [Kp_i; Kp_v; Ki_v];
    matlabFunction(gains, ...
         'Outputs', {'gains_expr'}, ...
         'File', "Cascade_gains", 'Optimize', false);
end