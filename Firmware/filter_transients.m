function [data_filt] = filter_transients(data, plot_bool, n_fig)
    arguments
        data
            plot_bool = false;
            n_fig = 1000;
    end
    
    len_data = length(data);
    data_filt = zeros(len_data, 1);
    idx_change_data = unique([0; find(ischange(data)); len_data]);
    
    % start_idxs = zeros(length(idx_change_data) - 1, 1);
    % end_idxs = zeros(length(idx_change_data), 1);
    for i = 1:length(idx_change_data)-1
        if idx_change_data(i + 1) - idx_change_data(i) > 100
            % start_idxs(i) = idx_change_data(i) + 1;
            % end_idxs(i) = idx_change_data(i + 1) - 1;
            idx_iter = (idx_change_data(i) + 1):(idx_change_data(i + 1) - 1);
            data_filt(idx_iter) = data(idx_iter);
        end
    end
    
    if plot_bool
        figure(n_fig)
        subplot(2, 1, 1)
        plot(data, 'lineWidth', 2); hold on;
        plot(data_filt, 'lineWidth', 2); hold on;
    end

    data_filt = data_filt(data_filt ~= 0);
end