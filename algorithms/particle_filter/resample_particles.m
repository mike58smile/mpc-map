function [new_particles] = resample_particles(particles, weights)
%RESAMPLE_PARTICLES Systematic resampling with light roughening.

N = size(particles, 1);

if N == 0
	new_particles = particles;
	return;
end

weights = weights(:);
if numel(weights) ~= N
	new_particles = particles;
	return;
end

weights(~isfinite(weights) | weights < 0) = 0;
w_sum = sum(weights);

if w_sum <= 0
	% Degenerate likelihoods should not destroy the cloud.
	weights = ones(N, 1) / N;
else
	weights = weights / w_sum;
end

new_particles = zeros(size(particles));

% Low-variance (systematic) resampling.
r = rand() / N;
c = weights(1);
i = 1;

for m = 1:N
	% Walk once through the cumulative distribution using evenly spaced draws.
	u = r + (m - 1) / N;
	while u > c && i < N
		i = i + 1;
		c = c + weights(i);
	end
	new_particles(m, :) = particles(i, :);
end

% Roughening to reduce particle impoverishment after duplication.
new_particles(:, 1) = new_particles(:, 1) + 0.003 * randn(N, 1);
new_particles(:, 2) = new_particles(:, 2) + 0.003 * randn(N, 1);
theta_noise = 0.01 * randn(N, 1);
new_particles(:, 3) = atan2(sin(new_particles(:, 3) + theta_noise), ...
						   cos(new_particles(:, 3) + theta_noise));

end

