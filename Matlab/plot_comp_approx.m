function [] = plot_comp_approx(un, un_approx)
    syms D T t real
    N_phases = length(un);

    figure(1);
    subplot(1, 2, 1)
    D_val = 0.5; % Example duty cycle
    T_val = 2*pi; % Example period
    
    hold on;
    for i = 1:N_phases
        % Substitute numerical values
        ui_plot = subs(un(i), [D, T], [D_val, T_val]);
        % Plot each PWM signal
        fplot(ui_plot, [0, T_val*(1 + (N_phases - 1)/(3*N_phases))], 'LineWidth', 2); % Plot 2 periods
    end
    hold off;
    xlabel('Time t');
    ylabel('PWM Amplitude');
    title('3-Phase PWM Signals');
    legend('Phase 1', 'Phase 2', 'Phase 3');
    grid on;
    
    % Plot the original PWM signal and its first-order Fourier approximation
    subplot(1, 2, 2)
    T_plot_end = T_val*(1 + (N_phases - 1)/(3*N_phases));
    hold on;
    for i = 1:1
        % Substitute numerical values
        ui_plot = subs(un(i), [D, T], [D_val, T_val]);
        % Plot each PWM signal
        fplot(ui_plot, [0, T_plot_end], 'LineWidth', 2); % Plot 2 periods

        % Plot each PWM signal approximation
        ui_approx_plot = subs(un_approx(i), [D, T], [D_val, T_val]);
        fplot(real(ui_approx_plot), [0, T_plot_end], '--', 'LineWidth', 2);
    end
    hold off;
    legend('PWM Signal u(t)', 'Approximation');
    xlabel('Time t');
    ylabel('PWM Amplitude');
    title('Comparison of PWM Signal and First-order Fourier Approximation');
    xlim([0, T_plot_end])
    grid on;
end    