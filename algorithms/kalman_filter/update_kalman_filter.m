function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)
%UPDATE_KALMAN_FILTER Summary of this function goes here

mu = public_vars.mu;
sigma = public_vars.sigma;

required_samples = 80;
if isfield(public_vars, 'kf') && isfield(public_vars.kf, 'gnss_init_required')
	required_samples = public_vars.kf.gnss_init_required;
end

theta_init_var = (2 * pi)^2;
if isfield(public_vars, 'kf') && isfield(public_vars.kf, 'theta_init_var')
	theta_init_var = public_vars.kf.theta_init_var;
end

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

if size(gnss_samples, 1) < required_samples
	if isempty(gnss_samples)
		mu_xy = mu(1:2).';
		cov_xy = public_vars.kf.Q;
	elseif size(gnss_samples, 1) == 1
		mu_xy = gnss_samples(1, :);
		cov_xy = public_vars.kf.Q;
	else
		mu_xy = mean(gnss_samples, 1);
		cov_xy = cov(gnss_samples);
	end

	mu = [mu_xy(1); mu_xy(2); 0.0];
	sigma = [cov_xy, [0; 0]; 0, 0, theta_init_var];
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

% II. Measurement
z = [];
if isfield(read_only_vars, 'gnss_position') && numel(read_only_vars.gnss_position) >= 2
	gnss_z = read_only_vars.gnss_position(1:2);
	if all(isfinite(gnss_z))
		z = gnss_z(:);
	end
end
[mu, sigma] = kf_correct(mu, sigma, z, public_vars.kf);

