function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)
%UPDATE_KALMAN_FILTER Summary of this function goes here

mu = public_vars.mu;
sigma = public_vars.sigma;

% Task 1: estimate initial belief from GNSS while the robot is stationary.
required_samples = 80;
if isfield(public_vars, 'kf') && isfield(public_vars.kf, 'gnss_init_required')
	required_samples = public_vars.kf.gnss_init_required;
end

gnss_samples = zeros(0, 2);
if isfield(read_only_vars, 'gnss_history') && ~isempty(read_only_vars.gnss_history)
	gnss_samples = read_only_vars.gnss_history(:, 1:2);
elseif isfield(read_only_vars, 'gnss_position') && numel(read_only_vars.gnss_position) >= 2
	gnss_samples = read_only_vars.gnss_position(1:2);
end

if ~isempty(gnss_samples)
	gnss_samples = gnss_samples(all(isfinite(gnss_samples), 2), :);
end

if size(gnss_samples, 1) < required_samples
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

	mu = [mu_xy(1); mu_xy(2); 0.0];
	sigma = [cov_xy, [0; 0]; 0, 0, (pi)^2];
	return;
end

if size(gnss_samples, 1) == required_samples
	mu_xy = mean(gnss_samples, 1);
	cov_xy = cov(gnss_samples);
	fprintf('GNSS initialization complete.\n');
	fprintf('Mean: [%.4f %.4f]\n', mu_xy(1), mu_xy(2));
	fprintf('Covariance matrix:\n');
	disp(cov_xy);
end

% I. Prediction
u = [];
[mu, sigma] = ekf_predict(mu, sigma, u, public_vars.kf, read_only_vars.sampling_period);

% II. Measurement
z = [];
[mu, sigma] = kf_measure(mu, sigma, z, public_vars.kf);

end

