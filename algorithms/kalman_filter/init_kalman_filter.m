function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

public_vars.kf.C = [1, 0, 0; 0, 1, 0];
% Process noise: keep position low for smoothness, heading slightly higher to adapt.
public_vars.kf.R = diag([0.0001, 0.0001, 0.0003]);
public_vars.kf.Q = [0.4980, -0.0046; -0.0046, 0.3586];
public_vars.kf.L = read_only_vars.agent_drive.interwheel_dist;
public_vars.kf.gnss_init_required = 120;
public_vars.kf.theta_init_var = (2 * pi)^2;

if isfield(read_only_vars, 'gnss_position') && numel(read_only_vars.gnss_position) >= 2 && all(isfinite(read_only_vars.gnss_position(1:2)))
	mu_xy = read_only_vars.gnss_position(1:2);
else
	mu_xy = [2.0, 2.0];
end

public_vars.mu = [mu_xy(1); mu_xy(2); 0.0];
public_vars.sigma = [public_vars.kf.Q, [0; 0]; 0, 0, public_vars.kf.theta_init_var];

end

