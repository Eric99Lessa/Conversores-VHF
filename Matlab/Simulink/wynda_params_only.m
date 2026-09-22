function [theta_hist, Gamma_hist] = wynda_params_only(x_f, Psi_func, opts)
% WYNDA_PARAMS_ONLY  Parameter-only WyNDA adaptive observer
%
% INPUTS:
%   x_f       - [n x N] matrix of pre-filtered states, each column is x_f(k)
%   Psi_func  - function handle: Psi_func(y, u) returns [n x p] basis matrix
%               where p = number of parameters
%   opts      - struct with fields:
%                 .y         [n x N]   measurement data
%                 .u         [m x N]   control input data ([] if none)
%                 .theta0    [p x 1]   initial parameter guess
%                 .P_theta0  [p x p]   initial covariance (symmetric, >0)
%                 .R_theta   [p x p]   or scalar — measurement noise tuning
%                 .Gamma0    [n x p]   initial Gamma (Gamma0'*Gamma0 > 0)
%                 .lambda    scalar    forgetting factor in (0,1)
%
% OUTPUTS:
%   theta_hist  - [p x N] estimated parameters over time
%   Gamma_hist  - [n x p x N] Gamma history (optional)

%% Unpack options
y       = opts.y;
u       = opts.u;
N       = size(x_f, 2);
n       = size(x_f, 1);

theta   = opts.theta0;
P_theta = opts.P_theta0;
Gamma   = opts.Gamma0;
lambda  = opts.lambda;

% Allow scalar R_theta
if isscalar(opts.R_theta)
    R_theta = opts.R_theta * eye(size(P_theta, 1));
else
    R_theta = opts.R_theta;
end

%% Preallocate
p           = length(theta);
theta_hist  = zeros(p, N);
Gamma_hist  = zeros(n, p, N);

%% Initial prediction
theta_pred = theta;   % theta(0|-1)

%% Main loop
for k = 2:N
    % --- Current basis matrix Psi(k-1) ---
    if isempty(u)
        uk = [];
    else
        uk = u(:, k-1);
    end
    Psi_km1 = Psi_func(y(:, k-1), uk);   % [n x p]

    % =========================================================
    % MEASUREMENT UPDATE
    % =========================================================

    % Innovation (prediction residual)
    x_pred  = x_f(:, k-1) + Psi_km1 * theta_pred;   % model prediction
    innov   = x_f(:, k) - x_pred;                    % [n x 1]

    % Omega
    Omega   = Gamma * P_theta * Gamma' + R_theta;     % [n x n]

    % Parameter gain
    K_theta = P_theta * Gamma' / Omega;               % [p x n]

    % Gamma update (simplified: no K_x)
    Gamma_upd = Gamma;                                % Gamma(k|k) = Gamma(k|k-1)

    % Parameter estimate update
    theta   = theta_pred - K_theta * innov;           % [p x 1]

    % Store
    theta_hist(:, k) = theta;
    Gamma_hist(:, :, k) = Gamma_upd;

    % =========================================================
    % PREDICTION UPDATE
    % =========================================================

    % Psi at current k (for Gamma propagation)
    if isempty(u)
        uk_curr = [];
    else
        uk_curr = u(:, k);
    end
    Psi_k = Psi_func(y(:, k), uk_curr);              % [n x p]

    % Parameter prediction (constant model)
    theta_pred = theta;                               % theta(k+1|k) = theta(k|k)

    % Covariance prediction
    P_theta = (1/lambda) * (eye(p) - K_theta * Gamma) * P_theta;

    % Gamma prediction
    Gamma   = Gamma_upd - Psi_k;                     % Gamma(k+1|k)
end

theta_hist(:, 1) = opts.theta0;   % fill k=0 with initial guess

end