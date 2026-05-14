function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Initialize a lightweight EKF for GNSS plus odometry.

public_vars.kf.C = [1, 0, 0; 0, 1, 0];
public_vars.kf.R = diag([0.008, 0.008, 0.02]);       % process noise
public_vars.kf.Q = diag([0.35 ^ 2, 0.35 ^ 2]);       % GNSS noise
public_vars.kf.L = read_only_vars.agent_drive.interwheel_dist;
public_vars.kf.theta_init_var = (2 * pi) ^ 2;

if isfield(read_only_vars, 'gnss_position') && numel(read_only_vars.gnss_position) >= 2 && ...
		all(isfinite(read_only_vars.gnss_position(1:2)))
	mu_xy = read_only_vars.gnss_position(1:2);
elseif isfield(public_vars, 'particles') && ~isempty(public_vars.particles)
	mu_xy = mean(public_vars.particles(:, 1:2), 1);
else
	mu_xy = [mean(read_only_vars.map.limits([1, 3])), mean(read_only_vars.map.limits([2, 4]))];
end

public_vars.mu = [mu_xy(1); mu_xy(2); 0.0];
public_vars.sigma = diag([1.0, 1.0, public_vars.kf.theta_init_var]);

end

