function K_vars = getGainVariables(cell_1, cell_2, cell_3)
%GETGAINVARIABLES Extract symbolic variables starting with capital K
%
%   K_vars = getGainVariables(cell_1, cell_2, cell_3)
%
%   Each input is a cell array. Each cell contains a struct with a field
%   called "eigenvalues". The function returns a unique list of symbolic
%   variables whose names start with capital K.

    all_cells = {cell_1, cell_2, cell_3};

    K_vars = sym.empty(1, 0);

    for c = 1:numel(all_cells)
        current_cell = all_cells{c};

        for i = 1:numel(current_cell)

            % Get struct inside the cell
            if iscell(current_cell)
                S = current_cell{i};
                eigvals = S.eigenvalues;
            else
                eigvals = current_cell;
            end
            % Get all symbolic variables in eigenvalues
            vars = symvar(eigvals);

            % Keep only variables whose names start with 'K'
            for j = 1:numel(vars)
                var_name = char(vars(j));

                if startsWith(var_name, 'K')
                    K_vars(end+1) = vars(j); %#ok<AGROW>
                end
            end
        end
    end

    % Remove duplicates
    K_vars = unique(K_vars);
end