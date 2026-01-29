function [t_filt, data_filt, means_data] = filt_step_conv(t, data, step_dot)
    data_smooth = smoothdata(data, "movmedian", 500);
    data_dot = [0; diff(data_smooth)./diff(t)];
    
    conv_data_step_dot = conv(data_dot, step_dot);

    filt_data = conv_data_step_dot(1:length(t)) == 0;
    data_filt = data(filt_data);
    t_filt = t(filt_data);

    idx_change = [0; find(diff(data_filt) > 50)] + 1;

    data_cell = cell(length(idx_change)-1, 1);
    means_data = zeros(length(idx_change)-1, 1);

    figure(5)
    for i = 1:length(idx_change)-1
        data_cell{i} = data_filt(idx_change(i)+1:idx_change(i + 1));
        means_data(i) = mean(rmoutliers(data_cell{i}));

        plot(t_filt(idx_change(i)+1:idx_change(i + 1)), data_cell{i}, 'lineWidth', 2); hold on;
        plot([t_filt(idx_change(i)+1); t(idx_change(i + 1))], [means_data(i); means_data(i)], 'lineWidth', 2);
    end
end