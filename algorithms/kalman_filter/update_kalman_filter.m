function [mu, sigma, kf] = update_kalman_filter(read_only_vars, public_vars)
%UPDATE_KALMAN_FILTER Summary of this function goes here

mu = public_vars.mu;
sigma = public_vars.sigma;
kf = public_vars.kf;

% Task 1: initialization from GNSS mean/covariance.
if isfield(kf, 'init') && ~kf.init.done
	if all(isfinite(read_only_vars.gnss_position))
		samples = [kf.init.gnss_samples; read_only_vars.gnss_position];
		kf.init.gnss_samples = samples;
	else
		samples = kf.init.gnss_samples;
	end

	n = size(samples, 1);
	if n >= 1
		mu_xy = mean(samples, 1);
	else
		mu_xy = mu(1:2);
	end

	if n >= kf.init.required_samples
		cov_xy = cov(samples, 1);
		cov_xy = cov_xy + 1e-6 * eye(2);

		mu = [mu_xy, pi/2];
		sigma = [cov_xy, zeros(2,1); 0, 0, pi^2];
		kf.Q = cov_xy;
		kf.init.done = true;

		fprintf('KF init done with %d GNSS samples. mean=[%.3f %.3f], cov=%s\n', ...
			n, mu_xy(1), mu_xy(2), mat2str(cov_xy, 4));
	else
		mu = [mu_xy, pi/2];
		sigma = diag([1.0, 1.0, pi^2]);
	end

	return;
end

% I. Prediction
u = [];
[mu, sigma] = ekf_predict(mu, sigma, u, kf, read_only_vars.sampling_period);

% II. Measurement
z = [];
[mu, sigma] = kf_measure(mu, sigma, z, kf);

end

