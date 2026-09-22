clear; clc; close all;

n_phases = 3; % Number of phases
order_approx = 0; % Approximation order
integ = 1;

dictionaries;

save_expr_conv = true;
save_expr = false;

%% 
d_num = sym(zeros(n_phases, 1));
for n = 1:n_phases
    d_num(n) = piecewise(sym(1) == sym(1), 0);
end
d_den = piecewise(sym(1) == sym(1), 1);
Kj = piecewise(sym(1) == sym(1), 1);
Kr = piecewise(sym(1) == sym(1), 1);
Ki = piecewise(sym(1) == sym(1), 0);

convs = ["Boost" "Buck"];
for conv = convs
    X_dot = interleavedConverterModel(n_phases, order_approx, conv);
    
    [num_conv, den_conv, Kj_conv, Kr_conv, Ki_conv] = controlLawIDAPBCExpr(X_dot, integ);

    [~, den_conv] = equivalentExpressions(den_conv);

    syms converter

    for n = 1:n_phases
        d_num(n) = piecewise(converter == map_conv(conv), num_conv(n), d_num(n));
    end
    d_den = piecewise(converter == map_conv(conv), den_conv, d_den);
    Kj = piecewise(converter == map_conv(conv), Kj_conv, Kj);
    Kr = piecewise(converter == map_conv(conv), Kr_conv, Kr);
    Ki = piecewise(converter == map_conv(conv), Ki_conv, Kr);

    if save_expr_conv
        file_name = "IDAPBC_duty_" + conv;
        matlabFunction(num_conv, den_conv, ...
         'Outputs', {'d_num', 'd_den'}, ...
         'File', file_name, 'Optimize', false);
    
        gains = [Kj_conv; Kr_conv; Ki_conv];
        file_name = "IDAPBC_gains_" + conv;
        matlabFunction(gains, ...
             'Outputs', {'gains_expr'}, ...
             'File', file_name, 'Optimize', false);
    end
end

%% Salva as expressoes (copie e cole)

if save_expr
    matlabFunction(d_num, d_den, ...
         'Outputs', {'d_num', 'd_den'}, ...
         'File', "IDAPBC_duty", 'Optimize', false); %#ok<*UNRCH>
    
    gains = [Kj; Kr; Ki];
    matlabFunction(gains, ...
         'Outputs', {'gains_expr'}, ...
         'File', "IDAPBC_gains", 'Optimize', false);
end