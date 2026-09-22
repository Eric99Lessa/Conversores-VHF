function [x_pred, P_pred, Xsigma_pred] = predict_step( ...
    x, P, Q, d, L, C, RL, VD, Ts, n_phases, lambda, Wm, Wc)

    n_states = length(x);
    n_sigma = 2*n_states + 1;

    %% Generate sigma points
    Xsigma = generateSigmaPoints(x, P, lambda);

    %% Propagate sigma points through process model
    Xsigma_pred = zeros(n_states, n_sigma);

    for i = 1:n_sigma
        Xsigma_pred(:, i) = boostConverterModel( ...
            Xsigma(:, i), d, L, C, RL, VD, Ts, n_phases);
    end

    %% Predicted mean
    x_pred = zeros(n_states, 1);

    for i = 1:n_sigma
        x_pred = x_pred + Wm(i)*Xsigma_pred(:, i);
    end

    %% Predicted covariance
    P_pred = Q;

    for i = 1:n_sigma
        dx = Xsigma_pred(:, i) - x_pred;
        P_pred = P_pred + Wc(i)*(dx*dx.');
    end

    %% Symmetrize covariance
    P_pred = 0.5*(P_pred + P_pred.');
end