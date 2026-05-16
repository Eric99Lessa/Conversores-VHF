function [eq_bool, expr_generic] = equivalentExpressions(expr_array)
    if isscalar(expr_array)
        eq_bool = true;
        expr_generic = expr_array;
        return
    end
    % Convert symbolic array to string, then strip brackets and split by ";"
    raw = string(expr_array);
    raw = strtrim(raw);
    raw = regexprep(raw, '^\[|\]$', '');  % remove leading [ and trailing ]
    exprs = strtrim(split(raw, ";"));      % split into array of strings
    eq_bool = true;
    for i = 1:length(exprs)-1
        eq_bool = eq_bool && strcmp(exprs(i), exprs(i+1));
    end
    if eq_bool
        expr_generic = str2sym(exprs(1));
        return;
    end
    % --- 1) Auto-discover "array" variables (name + numeric suffix) ---
    % Find all tokens matching <letters_and_underscores><digits>
    all_tokens = regexp(exprs, '[a-zA-Z_]+\d+', 'match');
    all_tokens = unique([all_tokens{:}]);  % flatten and deduplicate

    % Group by base name (strip trailing digits)
    bases = unique(regexprep(all_tokens, '\d+$', ''));

    % Keep only bases that actually appear with multiple different suffixes
    array_bases = {};
    for i = 1:numel(bases)
        matches = all_tokens(startsWith(all_tokens, bases{i}) & ...
                  ~cellfun(@isempty, regexp(all_tokens, ['^' bases{i} '\d+$'], 'match')));
        if numel(matches) > 1
            array_bases{end+1} = bases{i}; %#ok<AGROW>
        end
    end

    % --- 2) Normalize: replace <base><digits> -> <base>_IDX for each base ---
    normalized = exprs;
    for i = 1:numel(array_bases)
        b = array_bases{i};
        % Sort by length descending to avoid partial replacements (e.g. IL before IL_dot)
        normalized = regexprep(normalized, [b '\d+'], b);
    end

    eq_bool = all(normalized == normalized(1));
    
    expr_generic = expr_array;
    if eq_bool
        expr_generic = str2sym(normalized(1));
    end
end
