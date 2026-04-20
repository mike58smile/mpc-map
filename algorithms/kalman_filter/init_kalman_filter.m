function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

public_vars.kf.C = [1, 0, 0; 0, 1, 0];
public_vars.kf.R = diag([0.0003, 0.0003, 0.0002]);
public_vars.kf.Q = [0.2490, -0.0023; -0.0023, 0.1793];
public_vars.kf.L = read_only_vars.agent_drive.interwheel_dist;

public_vars.mu = [2.0; 2.0; pi/2];
public_vars.sigma = zeros(3, 3);

end

