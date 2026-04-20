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
u = [0.0, 0.0];
if isfield(public_vars, 'motion_vector') && numel(public_vars.motion_vector) >= 2
	vR = public_vars.motion_vector(1);
	vL = public_vars.motion_vector(2);
	if all(isfinite([vR, vL]))
		v = 0.5 * (vR + vL);
		omega = (vR - vL) / public_vars.kf.L;
		u = [v, omega];
	end
end
[mu, sigma] = ekf_predict(mu, sigma, u, public_vars.kf, read_only_vars.sampling_period);

% Store a-priori estimate before measurement correction.
mu_pred = mu;

% II. Measurement
z = [];
if isfield(read_only_vars, 'gnss_position') && numel(read_only_vars.gnss_position) >= 2
	gnss_z = read_only_vars.gnss_position(1:2);
	if all(isfinite(gnss_z))
		z = gnss_z(:);
	end
end
[mu, sigma] = kf_correct(mu, sigma, z, public_vars.kf);

% Print only first few post-init steps
if read_only_vars.counter <= (required_samples + 8)
    fprintf('k=%d\n', read_only_vars.counter);
	if ~isempty(z)
		fprintf('z=[%.3f %.3f]\n', z(1), z(2));
	else
		fprintf('z=[NaN NaN]\n');
	end
    fprintf('mu_pred=[%.3f %.3f %.3f]\n', mu_pred(1), mu_pred(2), mu_pred(3));
    fprintf('mu_corr=[%.3f %.3f %.3f]\n', mu(1), mu(2), mu(3));
    fprintf('diag(Sigma_corr)=[%.4f %.4f %.4f]\n\n', sigma(1,1), sigma(2,2), sigma(3,3));
end

