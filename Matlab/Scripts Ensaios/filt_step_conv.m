function [t_filt, data_filt, means_data] = filt_step_conv(t, data, step_dot)
    data_smooth = smoothdata(data, "movmedian", 100);
    data_dot = [0; diff(data_smooth)./diff(t)];
    
    conv_data_step_dot = conv(data_dot, step_dot);

    figure(4)
    plot(conv_data_step_dot); hold on;
    % plot(conv_data_step_dot.^2)

    filt_data = abs(conv_data_step_dot(1:length(t))) <= 10;
    data_filt = data(filt_data);
    t_filt = t(filt_data);

    idx_change = [0; find(diff(data_filt) > 50)] + 1;
    start_idx = idx_change(1:end);
    end_idx = [idx_change(2:end) - 1; length(t_filt)];

    for i = 1:length(idx_change)-1
        if end_idx(i) - start_idx(i) < 20
            if i ~= length(idx_change)-1
                start_idx(i) = start_idx(i + 1);
                end_idx(i) = end_idx(i + 1);
            else
                start_idx(i) = start_idx(i - 1);
                end_idx(i) = end_idx(i - 1);
            end
        end
    end
    start_idx = unique(start_idx);
    end_idx = unique(end_idx);

    data_cell = cell(length(start_idx), 1);
    means_data = zeros(length(start_idx), 1);

    figure(5)
    for i = 1:length(start_idx)
        data_cell{i} = data_filt(start_idx(i):end_idx(i));
        means_data(i) = mean(rmoutliers(data_cell{i}));

        plot(t_filt(start_idx(i):end_idx(i)), data_cell{i}, 'lineWidth', 2); hold on;
        plot([t_filt(start_idx(i)); t(end_idx(i))], [means_data(i); means_data(i)], 'lineWidth', 2);
    end
end