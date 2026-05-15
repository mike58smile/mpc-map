function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)
%UPDATE_KALMAN_FILTER Predict with wheel odometry and correct with GNSS.

mu = public_vars.mu;
sigma = public_vars.sigma;

% I. Prediction. The simulator stores wheel velocities as [vR, vL].
control = [0.0, 0.0];
if isfield(public_vars, 'motion_vector') && numel(public_vars.motion_vector) >= 2
	right_velocity = public_vars.motion_vector(1);
	left_velocity = public_vars.motion_vector(2);
	if all(isfinite([right_velocity, left_velocity]))
		linear_velocity = 0.5 * (right_velocity + left_velocity);
		angular_velocity = (right_velocity - left_velocity) / public_vars.kf.L;
		control = [linear_velocity, angular_velocity];
	end
end
[mu, sigma] = ekf_predict(mu, sigma, control, public_vars.kf, read_only_vars.sampling_period);

% II. Measurement update for x/y when GNSS is available.
measurement = [];
if isfield(read_only_vars, 'gnss_position') && numel(read_only_vars.gnss_position) >= 2
	gnss_position = read_only_vars.gnss_position(1:2);
	if all(isfinite(gnss_position))
		measurement = gnss_position(:);
	end
end
[mu, sigma] = kf_correct(mu, sigma, measurement, public_vars.kf);

% III. GNSS displacement supplies an occasional heading observation.
gnss_heading = heading_from_gnss_history(read_only_vars);
if isfinite(gnss_heading)
	blend = 0.35;
	mu(3) = angle_average(mu(3), gnss_heading, blend);
	sigma(3, 3) = min(sigma(3, 3), 0.25);
end

mu(3) = wrap_to_pi(mu(3));

end

function heading = heading_from_gnss_history(read_only_vars)
%HEADING_FROM_GNSS_HISTORY Estimate travel direction from recent GNSS drift.

heading = nan;
if ~isfield(read_only_vars, 'gnss_history') || size(read_only_vars.gnss_history, 1) < 2
	return;
end

history = read_only_vars.gnss_history(:, 1:2);
valid_rows = all(isfinite(history), 2);
history = history(valid_rows, :);
if size(history, 1) < 2
	return;
end

latest = history(end, :);
for history_index = size(history, 1) - 1:-1:1
	displacement = latest - history(history_index, :);
	if norm(displacement) > 0.22
		% Ignore tiny displacements because GNSS noise would dominate heading.
		heading = atan2(displacement(2), displacement(1));
		return;
	end
end
end

function angle = angle_average(old_angle, new_angle, new_weight)
%ANGLE_AVERAGE Blend angles through sine/cosine to avoid wrap discontinuity.

old_weight = 1 - new_weight;
angle = atan2(old_weight * sin(old_angle) + new_weight * sin(new_angle), ...
			  old_weight * cos(old_angle) + new_weight * cos(new_angle));
end

function angle = wrap_to_pi(angle)
%WRAP_TO_PI Local toolbox-free angle wrapping helper.

angle = mod(angle + pi, 2 * pi) - pi;
end

