function Xsigma = generateSigmaPoints(x, P, lambda)
    n_states = length(x);
    n_sigma = 2*n_states + 1;

    Xsigma = zeros(n_states, n_sigma);

    %% Numerical stabilization
    P = 0.5*(P + P.');
    P = P + 1e-9*eye(n_states);

    %% Cholesky factorization
    S = chol((n_states + lambda)*P, 'lower');

    %% Sigma points
    Xsigma(:, 1) = x;

    for i = 1:n_states
        Xsigma(:, i + 1) = x + S(:, i);
        Xsigma(:, i + 1 + n_states) = x - S(:, i);
    end
end