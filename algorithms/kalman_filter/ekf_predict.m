function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)
%EKF_PREDICT Differential-drive EKF prediction step.

if isempty(mu) || isempty(sigma)
	new_mu = mu;
	new_sigma = sigma;
	return;
end

if isempty(u) || numel(u) < 2 || any(~isfinite(u(1:2)))
	% Invalid control input means "do not move" rather than failing the filter.
	v = 0.0;
	omega = 0.0;
else
	v = u(1);
	omega = u(2);
end

dt = sampling_period;
theta = mu(3);

% Nonlinear state transition g(x,u) for differential-drive robot.
new_mu = [
	mu(1) + cos(theta) * v * dt;
	mu(2) + sin(theta) * v * dt;
	mu(3) + omega * dt
];

% Jacobian G_t = dg/dx evaluated at current state.
G = eye(3);
G(1,3) = -sin(theta) * v * dt;
G(2,3) =  cos(theta) * v * dt;

R = kf.R;
% Propagate covariance through the linearized motion model and add process
% noise for unmodeled slip and command uncertainty.
new_sigma = G * sigma * G.' + R;

% Keep heading in [-pi, pi] for numerical stability.
new_mu(3) = atan2(sin(new_mu(3)), cos(new_mu(3)));

end

