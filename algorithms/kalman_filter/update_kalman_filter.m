function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)
%UPDATE_KALMAN_FILTER Summary of this function goes here

mu = public_vars.mu;
sigma = public_vars.sigma;

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

