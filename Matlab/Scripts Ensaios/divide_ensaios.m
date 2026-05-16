function [data_cell] = divide_ensaios(data, threshold)
    idx_ensaios = [1; find(diff(data) < -threshold) + 1; length(data)];

    data_cell = cell(1, 1);
    for i = 1:length(idx_ensaios)-1
        data_cell{i} = data((idx_ensaios(i) + 1):(idx_ensaios(i + 1) - 1));
    end
end