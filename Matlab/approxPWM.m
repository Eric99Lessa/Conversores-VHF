function [basis, un_approx] = approxPWM(n_phases, order_approx, plota)
    arguments
        n_phases
        order_approx
        plota = false;
    end

    n_states_eq = 1 + 2*order_approx; % Number of states per equation

    %% PWM
    syms t T duty real
    syms d [n_phases 1]

    % Fundamental frequency
    omega0 = 2*pi/T;

    % Initialize basis
    basis = sym(ones(n_states_eq, 1));
    u_approx = (1/T)*int(1, t, 0, duty*T);
    for n = 1:order_approx
        u_approx = u_approx + (1/T)*int(exp(-1i*n*omega0*t), t, 0, d*T)*(exp(1i*n*omega0*t));
        u_approx = u_approx + (1/T)*int(exp(1i*n*omega0*t), t, 0, d*T)*(exp(-1i*n*omega0*t));
        basis(2*n) = cos(n*omega0*t);
        basis(2*n + 1) = sin(n*omega0*t);
    end
    u_approx = simplify(rewrite(u_approx,"sincos"));
    
    % Preallocate symbolic array for phase-shifted signals
    syms un_approx [n_phases 1]

    t_delay = ((1:n_phases) - 1)*T/n_phases;
    
    % Generate phase-shifted PWM signals
    for i = 1:n_phases
        un_approx(i) = subs(u_approx, t, t - t_delay(i));
        un_approx(i) = subs(un_approx(i), duty, d(i));
    end
    
    %% Plot original and approximation signals
    if plota
        syms un [n_phases 1]
        for i = 1:n_phases
            % Define the piecewise function for the PWM signal u(t)
            u = piecewise(0 <= t & t < d(i)*T, 1, d(i)*T <= t & t < T, 0);
            un(i) = subs(u, t, t - t_delay(i));
        end

        plot_comp_approx(un, un_approx);
    end
end