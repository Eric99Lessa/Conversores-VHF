function [x_pred_next, theta_pred_next, Px_next, Ptheta_next, Gamma_next] = ...
    wynda_predict(x_corr, theta_corr, Px, Ptheta, Gamma_corr, Psi, Kx, Ktheta, lambda_x, lambda_t)
%WYNDA_PREDICT Discrete prediction step of adaptive observer.
%
%   Model:
%       x(k+1) = x(k) + Psi(y(k),u(k))*theta
%
%   Inputs
%   ------
%   x_corr     : n-by-1 corrected state estimate at k
%   theta_corr : r-by-1 corrected parameter estimate at k
%   Px         : n-by-n state covariance-like matrix
%   Ptheta     : r-by-r parameter covariance-like matrix
%   Gamma_corr : n-by-r corrected sensitivity matrix
%   Psi        : n-by-r regression matrix Psi(y,u)
%   Kx         : n-by-n state correction gain
%   Ktheta     : r-by-n parameter correction gain
%   lambda_x   : state forgetting factor
%   lambda_t   : parameter forgetting factor
%
%   Outputs
%   -------
%   x_pred_next     : n-by-1 predicted state estimate for k+1
%   theta_pred_next : r-by-1 predicted parameter estimate for k+1
%   Px_next         : n-by-n updated Px
%   Ptheta_next     : r-by-r updated Ptheta
%   Gamma_next      : n-by-r updated Gamma

    n = size(Px, 1);
    r = size(Ptheta, 1);

    % State and parameter prediction
    x_pred_next = x_corr + Psi * theta_corr;
    theta_pred_next = theta_corr;

    % Covariance-like updates
    Px_next = (1 / lambda_x) * (eye(n) - Kx) * Px;

    Ptheta_next = (1 / lambda_t) * (eye(r) - Ktheta * Gamma_corr) * Ptheta;

    % Sensitivity update
    Gamma_next = Gamma_corr - Psi;

    % Numerical symmetrization
    Px_next = 0.5 * (Px_next + Px_next.');
    Ptheta_next = 0.5 * (Ptheta_next + Ptheta_next.');
end