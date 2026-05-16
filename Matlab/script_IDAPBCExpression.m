clear; clc; close all;

n_phases = 1; % Number of phases
order_approx = 0; % Approximation order

save_expr = true;
save_gain = false;

dictionaries;

%% 

d_num = piecewise(sym(1) == sym(1), 0);
d_den = piecewise(sym(1) == sym(1), 1);

convs = ["Boost"]; % "Buck" "BuckBoost"];
for conv = convs
    X_dot = interleavedConverterModel(n_phases, order_approx, conv);
    
    [num, den, jac_d, jac_mf, res_mf, eig_values] = controlLawIDAPBCExpr(X_dot);

    [~, num] = equivalentExpressions(num);

    syms converter

    d_num = piecewise(converter == map_conv(conv), num, d_num);
    d_den = piecewise(converter == map_conv(conv), den, d_den);

    % Expr_recover = eig_values;
    % recoverVarsExpr;
    % syms tau_d IL_ref real
    % 
    % Kr_sol = solve(tau_d == -1/(-Kr*R_L/L), Kr);
    % eig_values = subs(eig_values, Kr, Kr_sol);
    % eig_values = subs(eig_values, IL, repmat(IL_ref, size(IL)));
    % eig_values = subs(eig_values, VD, 0);
end

duty = d_num./d_den;
if save_expr
     matlabFunction(duty, 'File', "IDAPBC_expr", 'Optimize', false);
end

%% Lugar das Raizes

n_roots = length(eig_values);

isreal_eig = false(n_roots, 1);
for i = 1:n_roots
    isreal_eig(i) = isreal(eig_values(i));
end

real_roots = simplify(eig_values(isreal_eig));
real_roots = real_roots(real_roots ~= 0);
conj_roots = simplify(eig_values(~isreal_eig));

% syms Kr Kj Ki real
% assume([Kr > 1, Kj > 1]);
% 
% if mode_solv == 1
%     syms r real
%     assume([r > 0, r < 1]);
% 
%     coeffs_real_roots = coeffs(real_roots(1), Kr);
%     resistive_term = coeffs_real_roots(end)*Kr;
%     interconnection_term = coeffs_real_roots(1);
% 
%     Kr_sol = solve(r == (interconnection_term + resistive_term)/resistive_term, Kr);
%     Kr_sol = simplify(Kr_sol);
% 
%     if save_gain
%         if mode_solv == 0
%             solv_str = "pseudoInverse";
%         elseif mode_solv == 1
%             solv_str = "currentEquat";
%         end
% 
%         fname = sprintf('Kr_%dF_%s', n_phases, solv_str);
%         matlabFunction(Kr_sol, 'File', fname, 'Optimize', true);
%     end
% 
%     if ~isempty(conj_roots)
%         conj_roots_Kj = simplify(subs(conj_roots, Kr, Kr_sol), 1000);
% 
%         real_part = simplify(sum(conj_roots_Kj)/2, 1000);
%     end
% end