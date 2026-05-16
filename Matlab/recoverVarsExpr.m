%%
if exist('Expr_recover', 'var')
    vars = symvar(Expr_recover);
else
    vars = symvar(X_dot);
end

% Create them as individual symbols first
syms(vars(:));

positive_vars = [C L R_L VD Vin];
real_vars = setxor(vars, positive_vars);
assume(real_vars(:), 'real');
assume(positive_vars(:), 'positive');

% Group variables by base name and reconstruct arrays
var_names = arrayfun(@char, vars, 'UniformOutput', false);

pattern = '.*\d$';
mask = regexp(var_names, pattern, 'once');
vars_ending_with_number_names = var_names(~cellfun('isempty', mask));

% Find variables with numeric suffixes (e.g., Xn1, Xn2, ...)
[base_names, ~, idx_b] = unique(regexprep(vars_ending_with_number_names, '\d+$', ''));
len_symbols = histcounts(idx_b);

for i = 1:length(base_names)
    base = base_names{i};
    if base == 'X'
        eval([base ' = sym(''' base ''', [' num2str(length(X_dot)) ' 1], "real");']);
    else
        eval([base ' = sym(''' base ''', [' num2str(len_symbols(i)) ' 1], "real");']);
    end
end
