function [eqs] = approxSystemFourier(order_approx, LHS, RHS, basis)
    n_states_eq = 1 + 2*order_approx; % Number of states per equation
    n_eq = length(LHS);
    
    % syms coeffs_LHS coeffs_RHS [n_eq n_states_eq]
    
    coeffs_LHS = sym(nan(n_eq, n_states_eq));
    coeffs_RHS = sym(nan(n_eq, n_states_eq));
    for i = 1:n_eq
        sep_coeffs_LHS = LHS(i);
        sep_coeffs_RHS = RHS(i);
        for n_basis = n_states_eq:-1:1
            if n_basis == 1
                coeffs_LHS(i, n_basis) = sep_coeffs_LHS;
                coeffs_RHS(i, n_basis) = sep_coeffs_RHS;
            else
                sep_coeffs_LHS = coeffs(sep_coeffs_LHS, basis(n_basis));
                if length(sep_coeffs_LHS) > 1
                    coeffs_LHS(i, n_basis) = sep_coeffs_LHS(2);
                end
                sep_coeffs_LHS = sep_coeffs_LHS(1);
            
                sep_coeffs_RHS = coeffs(sep_coeffs_RHS, basis(n_basis));
                if length(sep_coeffs_RHS) > 1
                    coeffs_RHS(i, n_basis) = sep_coeffs_RHS(2);
                end
                sep_coeffs_RHS = sep_coeffs_RHS(1);
            end
        end
    end
    coeffs_LHS(isnan(coeffs_LHS)) = 0;
    coeffs_RHS(isnan(coeffs_RHS)) = 0;

    eqs = coeffs_LHS(:) == coeffs_RHS(:);
end