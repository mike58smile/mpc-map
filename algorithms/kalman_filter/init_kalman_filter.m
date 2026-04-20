function [public_vars] = init_kalman_filter(~, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

public_vars.kf.C = [1, 0, 0; 0, 1, 0];
public_vars.kf.R = diag([0.01, 0.01, 0.01]);
public_vars.kf.Q = diag([0.1, 0.1]);

% Task 1: gather GNSS samples while standing still and estimate mean/cov.
public_vars.kf.init.required_samples = 80;
public_vars.kf.init.gnss_samples = zeros(0, 2);
public_vars.kf.init.done = false;

public_vars.mu = [2.0, 2.0, pi/2];
public_vars.sigma = diag([1.0, 1.0, pi^2]);

end

