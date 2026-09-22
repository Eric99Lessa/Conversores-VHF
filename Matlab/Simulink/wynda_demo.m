clear; clc; close all;

Ts = 1e-3;
Tfinal = 15;
t = 0:Ts:Tfinal;
N = length(t);

m = 1.0;
k = 84;
b = 0.9;

x = zeros(2, N);
x(:, 1) = [1; 0];

y = zeros(2, N);
u = zeros(1, N);

% Control/input excitation
for i = 1:N
    u(i) = 5*sin(2*pi*0.7*t(i)) + 2*sin(2*pi*2.1*t(i));
end

% Simulate plant
for i = 1:N-1
    dx = zeros(2, 1);

    dx(1) = x(2, i);
    dx(2) = -(k/m)*x(1, i) - (b/m)*x(2, i) + (1/m)*u(i);

    x(:, i+1) = x(:, i) + Ts * dx;
end

% Add measurement noise
rng(1);
y(1, :) = x(1, :) + 0.005 * randn(1, N);
y(2, :) = x(2, :) + 0.02  * randn(1, N);

% Observer storage
xhat_hist = zeros(2, N);
theta_hist = zeros(5, N);
innovation_hist = zeros(2, N);

reset = true;

for i = 1:N
    [xhat_hist(:, i), theta_hist(:, i), innovation_hist(:, i)] = ...
        wynda_observer_block(y(:, i), u(i), reset);

    reset = false;
end

theta_true = [0;
              Ts;
             -Ts*k/m;
             -Ts*b/m;
              Ts/m];

disp('True theta:');
disp(theta_true.');

disp('Estimated final theta:');
disp(theta_hist(:, end).');

A_est_increment = [theta_hist(1, end), theta_hist(2, end);
                   theta_hist(3, end), theta_hist(4, end)];

B_est_increment = [0;
                   theta_hist(5, end)];

A_est = A_est_increment / Ts;
B_est = B_est_increment / Ts;

disp('Estimated continuous-time A:');
disp(A_est);

disp('Estimated continuous-time B:');
disp(B_est);

% Plots
figure('Color', 'w');

subplot(2,1,1);
plot(t, x(1,:), 'k', 'LineWidth', 1.5); hold on;
plot(t, y(1,:), '.', 'Color', [0.7 0.7 1.0], 'MarkerSize', 4);
plot(t, xhat_hist(1,:), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Position');
legend('True', 'Measured', 'Estimated');

subplot(2,1,2);
plot(t, x(2,:), 'k', 'LineWidth', 1.5); hold on;
plot(t, y(2,:), '.', 'Color', [0.7 0.7 1.0], 'MarkerSize', 4);
plot(t, xhat_hist(2,:), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Velocity');
legend('True', 'Measured', 'Estimated');

figure('Color', 'w');

for j = 1:5
    subplot(3,2,j);
    plot(t, theta_hist(j,:), 'b', 'LineWidth', 1.4); hold on;
    yline(theta_true(j), 'k--', 'LineWidth', 1.3);
    grid on;
    xlabel('Time [s]');
    ylabel(['\theta_', num2str(j)]);
    legend('Estimated', 'True');
end