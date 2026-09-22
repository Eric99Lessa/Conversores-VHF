function [xhat, theta_hat, innovation] = wynda_observer_block(y, u, reset)
%#codegen
%WYNDA_OBSERVER_BLOCK MATLAB Function Block implementation.
%
%   Inputs
%   ------
%   y     : 2-by-1 measured state vector
%   u     : scalar control input
%   reset : boolean reset signal
%
%   Outputs
%   -------
%   xhat       : 2-by-1 corrected state estimate
%   theta_hat  : 5-by-1 parameter estimate
%   innovation : 2-by-1 prediction error

    persistent x_pred theta_pred Px Ptheta Gamma initialized

    % Dimensions for this example
    n = 2;
    r = 5;

    % Tuning parameters
    lambda_x = 0.995;
    lambda_t = 0.999;

    Rx = diag([1e-4, 1e-3]);
    Rtheta = 1e-3 * eye(n);

    if isempty(initialized) || reset
        x_pred = zeros(n, 1);

        % Initial parameter estimate
        theta_pred = zeros(r, 1);

        % Initial covariance-like matrices
        Px = 0.1 * eye(n);
        Ptheta = 10 * eye(r);

        % Initial sensitivity
        Gamma = zeros(n, r);

        initialized = true;
    end

    % ==============================================================
    % UPDATE STEP
    % Uses current measurement y(k)
    % ==============================================================

    [x_corr, theta_corr, Px_current, Ptheta_current, Gamma_corr, Kx, Ktheta, innovation] = ...
        wynda_update(y, x_pred, theta_pred, Px, Ptheta, Gamma, Rx, Rtheta);

    % Output corrected estimates
    xhat = x_corr;
    theta_hat = theta_corr;

    % ==============================================================
    % PREDICTION STEP
    % Uses current y(k), current u(k), and corrected theta(k)
    % to prepare x_pred(k+1)
    % ==============================================================

    Psi = wynda_psi_msd_controlled(y, u);

    [x_pred_next, theta_pred_next, Px_next, Ptheta_next, Gamma_next] = ...
        wynda_predict(x_corr, theta_corr, Px_current, Ptheta_current, ...
                      Gamma_corr, Psi, Kx, Ktheta, lambda_x, lambda_t);

    % Store values for next sample
    x_pred = x_pred_next;
    theta_pred = theta_pred_next;
    Px = Px_next;
    Ptheta = Ptheta_next;
    Gamma = Gamma_next;
end