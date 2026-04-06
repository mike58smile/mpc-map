function [public_vars] = task3_covariance_matrix(public_vars)
%TASK3_COVARIANCE_MATRIX Builds LiDAR and GNSS covariance matrices for Task 3.

% Initialize Task 3 state.
if ~isfield(public_vars, 'task3')
    public_vars.task3.done = false;
end

% Task 3 depends on Task 2 measurements and sigmas.
if public_vars.task3.done || ~isfield(public_vars, 'task2') || ~isfield(public_vars.task2, 'done') || ~public_vars.task2.done
    return;
end

if ~isfield(public_vars.task2, 'lidar_samples') || ~isfield(public_vars.task2, 'gnss_samples')
    return;
end

lidar_samples = public_vars.task2.lidar_samples;
gnss_samples = public_vars.task2.gnss_samples;

% We need at least 2 samples for each sensor to compute covariance.
if size(lidar_samples, 1) < 2 || size(gnss_samples, 1) < 2
    return;
end

% Use MATLAB's internal cov with unbiased normalization (n-1).
cov_lidar = cov(lidar_samples);
cov_gnss = cov(gnss_samples);

public_vars.task3.cov_lidar = cov_lidar;
public_vars.task3.cov_gnss = cov_gnss;

% Verify expected dimensions.
public_vars.task3.lidar_size_ok = isequal(size(cov_lidar), [8, 8]);
public_vars.task3.gnss_size_ok = isequal(size(cov_gnss), [2, 2]);

% Verify diagonal equals sigma^2 from Task 2.
sigma_lidar_sq = (public_vars.task2.sigma_lidar).^2;
sigma_gnss_sq = (public_vars.task2.sigma_gnss).^2;

lidar_diag = diag(cov_lidar).';
gnss_diag = diag(cov_gnss).';

% Simple check: diagonal of covariance should match sigma^2 (within fixed tolerance).
tol = 1e-6;
public_vars.task3.lidar_diag_ok = max(abs(lidar_diag - sigma_lidar_sq)) <= tol;
public_vars.task3.gnss_diag_ok = max(abs(gnss_diag - sigma_gnss_sq)) <= tol;

public_vars.task3.done = true;

figure(22);
clf;
subplot(1, 2, 1);
imagesc(cov_lidar);
axis image;
colorbar;
title('LiDAR covariance (8x8)');
xlabel('Channel');
ylabel('Channel');

subplot(1, 2, 2);
imagesc(cov_gnss);
axis image;
colorbar;
title('GNSS covariance (2x2)');
xlabel('Axis');
ylabel('Axis');
sgtitle('Task 3: Sensor covariance matrices');

fprintf('Task 3 completed.\n');
fprintf('LiDAR covariance size: %dx%d (expected 8x8) -> %d\n', size(cov_lidar, 1), size(cov_lidar, 2), public_vars.task3.lidar_size_ok);
fprintf('GNSS covariance size: %dx%d (expected 2x2) -> %d\n', size(cov_gnss, 1), size(cov_gnss, 2), public_vars.task3.gnss_size_ok);
fprintf('LiDAR diag(cov) ~= sigma^2 -> %d\n', public_vars.task3.lidar_diag_ok);
fprintf('GNSS diag(cov) ~= sigma^2 -> %d\n', public_vars.task3.gnss_diag_ok);

end
