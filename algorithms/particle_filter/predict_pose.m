function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
%PREDICT_POSE Summary of this function goes here

new_pose = old_pose;

if numel(old_pose) < 3 || numel(motion_vector) < 2
	return;
end

v_r = motion_vector(1);
v_l = motion_vector(2);
L = read_only_vars.agent_drive.interwheel_dist;
dt = read_only_vars.sampling_period;

v = 0.5 * (v_r + v_l);
omega = (v_r - v_l) / L;

% Probabilistic motion model: sample noisy controls for each particle.
sigma_v = 0.03 + 0.08 * abs(v);
sigma_omega = 0.02 + 0.06 * abs(omega);
v = v + sigma_v * randn();
omega = omega + sigma_omega * randn();

x = old_pose(1);
y = old_pose(2);
theta = old_pose(3);

if abs(omega) < 1e-6
	x = x + v * dt * cos(theta);
	y = y + v * dt * sin(theta);
else
	x = x + (v / omega) * (sin(theta + omega * dt) - sin(theta));
	y = y - (v / omega) * (cos(theta + omega * dt) - cos(theta));
end

theta = atan2(sin(theta + omega * dt), cos(theta + omega * dt));

new_pose(1:3) = [x, y, theta];

end

