function [public_vars] = task4_normal_distribution(public_vars)
%TASK4_NORMAL_DISTRIBUTION Generates Task 4 normal pdf plots.

if ~isfield(public_vars, 'task4')
    public_vars.task4.done = false;
end

% Task 4 uses sigma values produced by Task 2.
if public_vars.task4.done || ~isfield(public_vars, 'task2') || ~isfield(public_vars.task2, 'done') || ~public_vars.task2.done
    return;
end

if ~isfield(public_vars.task2, 'sigma_lidar') || ~isfield(public_vars.task2, 'sigma_gnss')
    return;
end

mu = 0;
sigma_lidar_1 = public_vars.task2.sigma_lidar(1); % Use the first LiDAR channel as an example.
sigma_gnss_x = public_vars.task2.sigma_gnss(1);  % Use the GNSS X component as an example.

sigma_lidar_1 = max(sigma_lidar_1, eps);
sigma_gnss_x = max(sigma_gnss_x, eps);

% Plot over four standard deviations of the larger sensor uncertainty so
% both curves are visible on the same x-axis.
x_limit = 4 * max(sigma_lidar_1, sigma_gnss_x);
x = linspace(-x_limit, x_limit, 500);

% norm_pdf is the local toolbox-free implementation used for this assignment.
pdf_lidar_1 = norm_pdf(x, mu, sigma_lidar_1);
pdf_gnss_x = norm_pdf(x, mu, sigma_gnss_x);

public_vars.task4.x = x;
public_vars.task4.pdf_lidar_1 = pdf_lidar_1;
public_vars.task4.pdf_gnss_x = pdf_gnss_x;
public_vars.task4.sigma_lidar_1 = sigma_lidar_1;
public_vars.task4.sigma_gnss_x = sigma_gnss_x;
public_vars.task4.done = true;

figure(23);
clf;
plot(x, pdf_lidar_1, 'LineWidth', 2);
hold on;
plot(x, pdf_gnss_x, 'LineWidth', 2);
grid on;
xlabel('x');
ylabel('pdf(x)');
legend('LiDAR channel 1', 'GNSS X', 'Location', 'best');
title('Task 4: Normal pdf using measured sensor sigma');

fprintf('Task 4 completed.\n');
fprintf('mu = %.2f, sigma LiDAR ch1 = %.4f, sigma GNSS X = %.4f\n', mu, sigma_lidar_1, sigma_gnss_x);

end
