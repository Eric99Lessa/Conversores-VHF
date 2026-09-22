function [x_corr, theta_corr, Px, Ptheta, Gamma_corr, Kx, Ktheta, innovation] = ...
    wynda_update(y, x_pred, theta_pred, Px, Ptheta, Gamma, Rx, Rtheta)
%WYNDA_UPDATE Discrete measurement update step of adaptive observer.
%
%   Inputs
%   ------
%   y          : n-by-1 measured state/output
%   x_pred    : n-by-1 predicted state estimate
%   theta_pred : r-by-1 predicted parameter estimate
%   Px         : n-by-n state covariance-like matrix
%   Ptheta     : r-by-r parameter covariance-like matrix
%   Gamma      : n-by-r sensitivity matrix
%   Rx         : n-by-n measurement noise/tuning matrix for state update
%   Rtheta     : n-by-n tuning matrix for parameter update
%
%   Outputs
%   -------
%   x_corr     : n-by-1 corrected state estimate
%   theta_corr : r-by-1 corrected parameter estimate
%   Px         : n-by-n unchanged current Px
%   Ptheta     : r-by-r unchanged current Ptheta
%   Gamma_corr : n-by-r corrected sensitivity matrix
%   Kx         : n-by-n state gain
%   Ktheta     : r-by-n parameter gain
%   innovation : n-by-1 prediction error y - x_pred

    n = length(y);

    % Innovation
    innovation = y - x_pred;

    % State correction gain
    Kx = Px / (Px + Rx);

    % Parameter correction gain
    Omega = Gamma * Ptheta * Gamma.' + Rtheta;

    % Safer than explicit inverse
    Ktheta = Ptheta * Gamma.' / Omega;

    % Corrected sensitivity
    Gamma_corr = (eye(n) - Kx) * Gamma;

    % State correction
    x_corr = x_pred + (Kx + Gamma_corr * Ktheta) * innovation;

    % Parameter correction
    %
    % The negative sign follows the observer convention used in the paper,
    % where Gamma represents sensitivity of the state-estimation error.
    theta_corr = theta_pred - Ktheta * innovation;
end