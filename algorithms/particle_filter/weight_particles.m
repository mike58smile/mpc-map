function [weights] = weight_particles(particle_measurements, lidar_distances, max_range)
%WEIGHT_PARTICLES Convert lidar residuals to normalized likelihood weights.
% Infinite ranges mean "no wall hit". Capping them at the map diagonal lets
% the filter penalize a predicted wall where the real scan saw open space.

particle_count = size(particle_measurements, 1);
if particle_count == 0
	weights = [];
	return;
end

if nargin < 3 || ~isfinite(max_range) || max_range <= 0
	% Fallback cap for infinite lidar ranges if the caller did not provide the
	% map diagonal.
	finite_values = [particle_measurements(isfinite(particle_measurements)); lidar_distances(isfinite(lidar_distances))];
	if isempty(finite_values)
		max_range = 10;
	else
		max_range = max(10, max(finite_values));
	end
end

observed = lidar_distances(:).';
beam_count = min(size(particle_measurements, 2), numel(observed));
if beam_count == 0
	weights = ones(particle_count, 1) / particle_count;
	return;
end

observed = observed(1:beam_count);
% Replace "no hit" with a large finite distance so open space can be compared
% numerically against predicted open space.
observed(~isfinite(observed)) = max_range;
observed = min(observed, max_range);

log_weights = zeros(particle_count, 1);
for particle_index = 1:particle_count
	predicted = particle_measurements(particle_index, 1:beam_count);
	predicted(~isfinite(predicted)) = max_range;
	predicted = min(predicted, max_range);

	% Farther beams are less precise, so their allowed residual is slightly
	% larger than for short-range returns.
	beam_sigma = 0.30 + 0.04 * observed;
	residual = predicted - observed;
	log_weights(particle_index) = -0.5 * sum((residual ./ beam_sigma) .^ 2);
end

max_log_weight = max(log_weights);
if ~isfinite(max_log_weight)
	weights = ones(particle_count, 1) / particle_count;
	return;
end

weights = exp(log_weights - max_log_weight);
weight_sum = sum(weights);
if weight_sum <= 0 || ~isfinite(weight_sum)
	weights = ones(particle_count, 1) / particle_count;
else
	weights = weights / weight_sum;
end

end

