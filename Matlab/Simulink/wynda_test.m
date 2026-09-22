%% ---- Simulate noisy MSD data ----
dt   = 1e-3;
t    = 0:dt:5;
N    = length(t);
n    = 2;   % states: [x; v]

% True parameters: m=1, k=84, b=0.9  =>  theta = [-k/m; -b/m] = [-84; -0.9]
A_true = [0, 1; -84, -0.9];
x_true = zeros(2, N);
x_true(:,1) = [1; 0];
for k = 1:N-1
    x_true(:,k+1) = x_true(:,k) + dt * A_true * x_true(:,k);
end

% Noisy measurements
y = x_true + 0.0 * randn(2, N);

% Assume x_f = y (or use your own Kalman-filtered states)
x_f = y;

%% ---- Basis function: Psi(y, u) for linear MSD ----
% Model: x(k+1) = x(k) + dt * [v; theta1*x + theta2*v]
% => Psi = [0, 0; dt*y(1), dt*y(2)]  for 2 params [theta1=-84, theta2=-0.9]
Psi_func = @(y, u) [0,          0;
                    dt*y(1),  dt*y(2)];

%% ---- Options ----
opts.y       = y;
opts.u       = [];
opts.theta0  = zeros(2, 1);
opts.P_theta0 = 0.1 * eye(2);
opts.R_theta  = eye(2);
opts.Gamma0   = 0.1 * eye(2);
opts.lambda   = 0.999;

%% ---- Run ----
[theta_hist, ~] = wynda_params_only(x_f, Psi_func, opts);

%% ---- Plot ----
figure;
subplot(2,1,1)
plot(t, theta_hist(1,:), 'g--', 'LineWidth', 1.5); hold on
yline(-84, 'k-', 'LineWidth', 1.5);
legend('Estimated \theta_1', 'True (-84)'); ylabel('\theta_1'); grid on;

subplot(2,1,2)
plot(t, theta_hist(2,:), 'g--', 'LineWidth', 1.5); hold on
yline(-0.9, 'k-', 'LineWidth', 1.5);
legend('Estimated \theta_2', 'True (-0.9)'); ylabel('\theta_2');
xlabel('t (s)'); grid on;