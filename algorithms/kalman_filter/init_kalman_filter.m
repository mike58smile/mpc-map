function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

public_vars.kf.C = [1, 0, 0; 0, 1, 0];
public_vars.kf.R = diag([0.10, 0.10, deg2rad(10)].^2);
public_vars.kf.Q = diag([0.35, 0.35].^2);
public_vars.kf.L = read_only_vars.agent_drive.interwheel_dist;
public_vars.kf.gnss_init_required = 80;

gnss_samples = zeros(0, 2);
if isfield(read_only_vars, 'gnss_history') && ~isempty(read_only_vars.gnss_history)
	gnss_samples = read_only_vars.gnss_history(:, 1:2);
elseif isfield(read_only_vars, 'gnss_position') && numel(read_only_vars.gnss_position) >= 2
	gnss_samples = read_only_vars.gnss_position(1:2);
end

if ~isempty(gnss_samples)
	valid_rows = all(isfinite(gnss_samples), 2);
	gnss_samples = gnss_samples(valid_rows, :);
end

if isempty(gnss_samples)
	mu_xy = [2.0, 2.0];
	cov_xy = diag([1.0, 1.0]);
elseif size(gnss_samples, 1) == 1
	mu_xy = gnss_samples(1, :);
	cov_xy = diag([0.35, 0.35].^2);
else
	mu_xy = mean(gnss_samples, 1);
	cov_xy = cov(gnss_samples);
end

public_vars.kf.gnss_init_mean = mu_xy;
public_vars.kf.gnss_init_cov = cov_xy;
public_vars.kf.gnss_init_samples = size(gnss_samples, 1);

public_vars.mu = [mu_xy(1); mu_xy(2); 0.0];
public_vars.sigma = [cov_xy, [0; 0]; 0, 0, (pi)^2];

end

