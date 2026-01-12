function [] = rlocus_2gains(roots_sym, Kj_range, Kr_range, params)
    arguments
        roots_sym sym
        Kj_range (1,:) double
        Kr_range (1,:) double
        params struct
    end
    % Vamos separar as raízes que são sempre reais das que podem ser complexas

    isreal_eig = false(n_eq, 1);
    for i = 1:n_eq
        isreal_eig(i) = isreal(roots(i));
    end
end