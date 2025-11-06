function [newExpression] = subsExpression(oldExpression, old_vars, new_vars)
    n_exp = numel(oldExpression);  % Use numel to avoid scalar indexing issues
    n_vars = numel(new_vars);  

    newExpression = oldExpression;  % Initialize with the input expression

    for idx_exp = 1:n_exp
        expr = oldExpression(idx_exp);  % Extract current expression
        for idx_vars = 1:n_vars
            expr = subs(expr, old_vars(idx_vars), new_vars(idx_vars));
        end
        newExpression(idx_exp) = expr;  % Store modified expression
    end
end
