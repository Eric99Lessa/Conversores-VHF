function [first_order, second_order, third_order] = classify_eigenvalues(eigs_list)  
    first_order = [];
    second_order = {};
    third_order = {};

    n_2nd_order = 0;
    n_3rd_order = 0;
    n_eig = length(eigs_list);

    used = false(1, n_eig);
    n = 1;
    idx_eig = 1:n_eig;
    while n <= n_eig
        if used(n), continue; end
        eig_n = eigs_list(n);

        has_pos_cuberoot = hasFractionalPower(eig_n, sym(1)/3);
        has_neg_cuberoot = hasFractionalPower(eig_n, -sym(1)/3);
    
        if ~has_pos_cuberoot && ~has_neg_cuberoot
            eig_current = expand(eig_n);
        else
            eig_current = eig_n;
        end

        childrens = children(eig_current);
        n_childrens = length(childrens);

        has_sqrt_child = false(1, n_childrens);
        has_cuberoot_child = false(1, n_childrens);
        has_inv_cuberoot_child = false(1, n_childrens);
        
        for n_c = 1:n_childrens
            child = childrens{n_c};
        
            has_sqrt_child(n_c) = hasFractionalPower(child, sym(1)/2);
            has_cuberoot_child(n_c) = hasFractionalPower(child, sym(1)/3);
            has_inv_cuberoot_child(n_c) = hasFractionalPower(child, -sym(1)/3);
        end

        if sum(has_sqrt_child) == 1
            % assuming a is 1
            b = -2*simplify(eig_current - childrens{has_sqrt_child});
            sqr_delta = 2*simplify(eig_current + b/2);
            c = simplify((b^2 - sqr_delta^2)/4);
            r_plus_sign = simplify((-b + sqrt(expand(b^2) - 4*c))/2);
            r_minus_sign = simplify((-b - sqrt(expand(b^2) - 4*c))/2);

            eigs_rem = eigs_list(~used);
            idx_eig_rem = idx_eig(~used);
            n_eig_rem = sum(~used);
            n_eig_founded = 0;
            for n2 = 1:n_eig_rem
                if ~used(idx_eig_rem(n2)) && isAlways(eigs_rem(n2) == r_plus_sign, Unknown="false")
                    used(idx_eig_rem(n2)) = true;
                    n_eig_founded = n_eig_founded + 1;
                elseif ~used(idx_eig_rem(n2)) && isAlways(eigs_rem(n2) == r_minus_sign, Unknown="false")
                    used(idx_eig_rem(n2)) = true;
                    n_eig_founded = n_eig_founded + 1;
                end
            end
            mult_pair = n_eig_founded/2;

            pair.eigenvalues = [r_plus_sign; r_minus_sign];
            pair.sum = -b;
            pair.product = c;
            for i = 1:mult_pair
                second_order{n_2nd_order + 1} = pair; %#ok<AGROW>
                n_2nd_order = n_2nd_order + 1;
            end
            n = n + 2*mult_pair;
            continue;
        elseif any(has_cuberoot_child) && any(has_inv_cuberoot_child)
            u = childrens{has_cuberoot_child};
            v = childrens{has_inv_cuberoot_child};

            A = -3*(eig_current - u - v);
            p = simplify(-3*u*v);
            q = -simplify(u^3 + v^3);
            a0 = (A/3)^3 + A*p/3 + q;
            r_real = u + v - A/3;

            omega = (-1 + sqrt(3)*1j)/2;
            omega2 = (-1 - sqrt(3)*1j)/2;
            r_conj1 = omega*u + omega2*v - A/3;
            r_conj2 = omega2*u + omega*v - A/3;

            eigs_rem = eigs_list(~used);
            idx_eig_rem = idx_eig(~used);
            n_eig_rem = sum(~used);
            for n2 = 1:n_eig_rem
                if ~used(idx_eig_rem(n2)) && isAlways(eigs_rem(n2) == r_real, Unknown="false")
                    used(idx_eig_rem(n2)) = true;
                elseif ~used(idx_eig_rem(n2)) && isAlways(eigs_rem(n2) == r_conj1, Unknown="false")
                    used(idx_eig_rem(n2)) = true;
                elseif ~used(idx_eig_rem(n2)) && isAlways(eigs_rem(n2) == r_conj2, Unknown="false")
                    used(idx_eig_rem(n2)) = true;
                end
            end
            
            triple.eigenvalues = [r_real; r_conj1; r_conj2];
            triple.eig_real = r_real;
            triple.sum = -A;
            triple.product = -a0;
            triple.p = p;
            triple.q = q;
            third_order{n_3rd_order + 1} = triple; %#ok<AGROW>
            n_3rd_order = n_3rd_order + 1;
            n = n + 3;
            continue;
        else
            used(n) = true;
            first_order = [first_order; eig_n]; %#ok<AGROW>
            n = n + 1;
            continue;
        end
    end
end