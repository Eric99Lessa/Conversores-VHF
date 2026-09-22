function [sol] = paramGains(first_order, second_order, third_order)
    K_vars = getGainVariables(first_order, second_order, third_order);

    %% Parametrize performance equations as function of zeta, settling time and settling time ratio
    if ~isempty(third_order)
        syms s
        syms zeta ts k positive
        assumeAlso(k < 0.5)

        p1 = -8*k/((1-k)*ts);
        p2 = -4/ts + 1i*(4/(zeta*ts))*sqrt(1 - zeta^2);
        p3 = -4/ts - 1i*(4/(zeta*ts))*sqrt(1 - zeta^2);
        
        poly_d   = expand((s - p1)*(s - p2)*(s - p3));
        poly_d   = collect(poly_d, s);
        coeffs_d = coeffs(poly_d, s, 'All');
        a2 = -third_order{1}.sum;
        a1 = (a2^2)/3 + third_order{1}.p;
        a0 = -third_order{1}.product;

        eq_a2      = coeffs_d(2) == a2;
        eq_a1 = coeffs_d(3) == a1;
        eq_a0      = coeffs_d(4) == a0;

        eqs = [eq_a2; eq_a1; eq_a0];
        sol = solve(eqs, K_vars);  
    elseif ~isempty(second_order)
        syms s
        syms zeta ts positive

        p1 = -4/ts + 1i*(4/(zeta*ts))*sqrt(1-zeta^2);
        p2 = -4/ts - 1i*(4/(zeta*ts))*sqrt(1-zeta^2);

        poly_d = expand((s - p1)*(s - p2));
        poly_d = collect(poly_d, s);
        coeffs_d = coeffs(poly_d, s, 'All');

        sum = second_order{1}.sum;
        product = second_order{1}.product;

        eqs = coeffs_d(2:end)' == [-sum; product];

        sol = solve(eqs, K_vars);
    elseif ~isempty(first_order)
        syms ts positive

        p1 = -4/ts;

        eqs = first_order(1) == p1;
        sol = solve(eqs, K_vars);
    end
end