function [x_upd, P_upd] = update_step( ...
    x_pred, P_pred, Xsigma_pred, y_meas, R_meas, Wm, Wc, n_phases)

    n_states = length(x_pred);
    n_sigma = size(Xsigma_pred, 2);
    n_meas = n_phases + 2;

    %% Propagate predicted sigma points through measurement model
    Ysigma = zeros(n_meas, n_sigma);

    for i = 1:n_sigma
        Ysigma(:, i) = measurementModel(Xsigma_pred(:, i), n_phases);
    end

    %% Predicted measurement mean
    y_pred = zeros(n_meas, 1);

    for i = 1:n_sigma
        y_pred = y_pred + Wm(i)*Ysigma(:, i);
    end

    %% Innovation covariance
    Pyy = R_meas;

    for i = 1:n_sigma
        dy = Ysigma(:, i) - y_pred;
        Pyy = Pyy + Wc(i)*(dy*dy.');
    end

    %% Cross covariance
    Pxy = zeros(n_states, n_meas);

    for i = 1:n_sigma
        dx = Xsigma_pred(:, i) - x_pred;
        dy = Ysigma(:, i) - y_pred;
        Pxy = Pxy + Wc(i)*(dx*dy.');
    end

    %% Kalman gain
    K = Pxy / Pyy;

    %% Correction
    x_upd = x_pred + K*(y_meas - y_pred);

    %% Covariance update
    P_upd = P_pred - K*Pyy*K.';

    %% Symmetrize covariance
    P_upd = 0.5*(P_upd + P_upd.');
end