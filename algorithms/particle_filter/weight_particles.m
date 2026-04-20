function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here

N = size(particle_measurements, 1);

if N == 0
	weights = [];
	return;
end

z = lidar_distances(:)';
sigma = 0.28;

log_weights = -inf(N, 1);
for i = 1:N
	z_hat = particle_measurements(i, :);
	valid = isfinite(z_hat) & isfinite(z);

	if ~any(valid)
		continue;
	end

	err = z_hat(valid) - z(valid);
	log_weights(i) = -0.5 * sum((err / sigma).^2);
end

max_log_weight = max(log_weights);
if ~isfinite(max_log_weight)
	weights = ones(N, 1) / N;
	return;
end

weights = exp(log_weights - max_log_weight);
weight_sum = sum(weights);

if weight_sum <= 0 || ~isfinite(weight_sum)
	weights = ones(N, 1) / N;
else
	weights = weights / weight_sum;
end

end

