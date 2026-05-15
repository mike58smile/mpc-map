function [particles] = update_particle_filter(read_only_vars, public_vars)
%UPDATE_PARTICLE_FILTER Predict, score, resample, and recover particles.
% The filter uses lidar everywhere and tightens x/y with GNSS whenever GNSS
% is available. No MoCap or toolbox functions are used.

particles = public_vars.particles;
if isempty(particles)
    return;
end

particle_count = size(particles, 1);
if size(particles, 2) < 4
    particles(:, 4) = 1 / particle_count;
end

% I. Prediction from the previously commanded wheel velocities.
for particle_index = 1:particle_count
    particles(particle_index, 1:3) = predict_pose( ...
        particles(particle_index, 1:3), public_vars.motion_vector, read_only_vars);
end

gnss_position = [nan, nan];
if isfield(read_only_vars, 'gnss_position') && numel(read_only_vars.gnss_position) >= 2
    gnss_position = read_only_vars.gnss_position(1:2);
end
gnss_available = all(isfinite(gnss_position));

if should_skip_lidar_correction(read_only_vars, gnss_available)
    % On GNSS-available frames the EKF can carry position between expensive
    % particle lidar updates. In GNSS-denied areas this never skips.
    return;
end

% Particles outside the arena or too close to walls get zero likelihood.
valid_particle = particles_are_valid(read_only_vars.map, particles(:, 1:2), 0.04);

% II. Lidar correction. Infinite ranges are handled inside weight_particles.
measurements = zeros(particle_count, length(read_only_vars.lidar_config));
for particle_index = 1:particle_count
    measurements(particle_index, :) = compute_lidar_measurement( ...
        read_only_vars.map, particles(particle_index, 1:3), read_only_vars.lidar_config);
end

map_diagonal = hypot(read_only_vars.map.limits(3) - read_only_vars.map.limits(1), ...
                     read_only_vars.map.limits(4) - read_only_vars.map.limits(2));
weights = weight_particles(measurements, read_only_vars.lidar_distances, map_diagonal);

% III. GNSS correction for position when the receiver is not denied.
if gnss_available
    gnss_sigma = 0.45;
    delta = particles(:, 1:2) - gnss_position;
    gnss_log_weights = -0.5 * sum(delta .^ 2, 2) / (gnss_sigma ^ 2);
    gnss_log_weights = gnss_log_weights - max(gnss_log_weights);
    gnss_weights = exp(gnss_log_weights);
    weights = weights .* gnss_weights;
end

weights(~valid_particle) = 0;
weights = normalize_weights(weights);
particles(:, 4) = weights;

effective_count = 1 / max(eps, sum(weights .^ 2));
if effective_count < 0.65 * particle_count
	% Resample only when the cloud has effectively collapsed to a small number
	% of particles; otherwise keep diversity without unnecessary duplication.
    particles = resample_particles(particles, weights);
    particles(:, 4) = 1 / particle_count;
end

% IV. A small adaptive reinjection prevents permanent localization loss.
best_lidar_error = best_measurement_error(measurements, read_only_vars.lidar_distances, map_diagonal);
reinject_ratio = 0.02;
if best_lidar_error > 0.9
    reinject_ratio = 0.25;
elseif effective_count < 0.25 * particle_count
    reinject_ratio = 0.14;
elseif effective_count < 0.45 * particle_count
    reinject_ratio = 0.08;
end

reinjected_count = min(particle_count, max(1, round(reinject_ratio * particle_count)));
reinjected_indices = randperm(particle_count, reinjected_count);
for index = 1:reinjected_count
	% Random reinjection is the recovery mechanism for wrong-map-mode lock-in
	% and kidnapped-robot-like localization loss.
    particles(reinjected_indices(index), 1:3) = sample_particle(read_only_vars.map, gnss_position);
    particles(reinjected_indices(index), 4) = 1 / particle_count;
end

particles(:, 4) = normalize_weights(particles(:, 4));

end

function weights = normalize_weights(weights)
%NORMALIZE_WEIGHTS Sanitize and normalize a likelihood vector.

weights = weights(:);
weights(~isfinite(weights) | weights < 0) = 0;
weight_sum = sum(weights);
if weight_sum <= 0
    weights = ones(size(weights)) / numel(weights);
else
    weights = weights / weight_sum;
end
end

function skip = should_skip_lidar_correction(read_only_vars, gnss_available)
%SHOULD_SKIP_LIDAR_CORRECTION Throttle ray-casting only when GNSS is present.

skip = false;
if ~isfield(read_only_vars, 'counter') || read_only_vars.counter <= 40
    return;
end

% GNSS can carry position between lidar updates. Without GNSS, every scan is
% important because the particle filter is the only absolute pose source.
if ~gnss_available
    return;
end
skip = mod(read_only_vars.counter, 3) ~= 0;
end

function error_value = best_measurement_error(measurements, lidar_distances, max_range)
%BEST_MEASUREMENT_ERROR Best mean absolute lidar residual among particles.

if isempty(measurements)
    error_value = inf;
    return;
end

observed = lidar_distances(:).';
observed(~isfinite(observed)) = max_range;
predicted = measurements;
predicted(~isfinite(predicted)) = max_range;
beam_count = min(size(predicted, 2), numel(observed));
if beam_count == 0
    error_value = inf;
    return;
end

residual = abs(predicted(:, 1:beam_count) - observed(1:beam_count));
error_value = min(mean(residual, 2));
end

function valid = particles_are_valid(map, xy, margin)
%PARTICLES_ARE_VALID Vector of particles inside bounds and clear of walls.

valid = xy(:, 1) > map.limits(1) + margin & xy(:, 1) < map.limits(3) - margin & ...
        xy(:, 2) > map.limits(2) + margin & xy(:, 2) < map.limits(4) - margin;
for wall_index = 1:size(map.walls, 1)
    wall_start = map.walls(wall_index, 1:2);
    wall_end = map.walls(wall_index, 3:4);
    for particle_index = 1:size(xy, 1)
        if valid(particle_index) && ...
                point_segment_distance(xy(particle_index, :), wall_start, wall_end) < margin
            valid(particle_index) = false;
        end
    end
end
end

function particle = sample_particle(map, gnss_position)
%SAMPLE_PARTICLE Draw a valid recovery particle, GNSS-biased when possible.

use_gnss = numel(gnss_position) >= 2 && all(isfinite(gnss_position(1:2)));

for attempt = 1:220
    if use_gnss
        x_pos = gnss_position(1) + 0.65 * randn();
        y_pos = gnss_position(2) + 0.65 * randn();
    else
        x_pos = map.limits(1) + rand() * (map.limits(3) - map.limits(1));
        y_pos = map.limits(2) + rand() * (map.limits(4) - map.limits(2));
    end

    if is_pose_valid(map, x_pos, y_pos, 0.08)
        particle = [x_pos, y_pos, -pi + 2 * pi * rand()];
        return;
    end

    if attempt == 100
        use_gnss = false;
    end
end

particle = [mean(map.limits([1, 3])), mean(map.limits([2, 4])), -pi + 2 * pi * rand()];
end

function valid = is_pose_valid(map, x_pos, y_pos, margin)
%IS_POSE_VALID Scalar version used by random particle sampling.

valid = x_pos > map.limits(1) + margin && x_pos < map.limits(3) - margin && ...
        y_pos > map.limits(2) + margin && y_pos < map.limits(4) - margin;
if ~valid
    return;
end

for wall_index = 1:size(map.walls, 1)
    if point_segment_distance([x_pos, y_pos], map.walls(wall_index, 1:2), ...
            map.walls(wall_index, 3:4)) < margin
        valid = false;
        return;
    end
end
end

function distance = point_segment_distance(point, segment_start, segment_end)
%POINT_SEGMENT_DISTANCE Distance from one point to one wall segment.

segment = segment_end - segment_start;
denominator = dot(segment, segment);
if denominator < eps
    distance = norm(point - segment_start);
    return;
end

projection = dot(point - segment_start, segment) / denominator;
projection = max(0, min(1, projection));
closest = segment_start + projection * segment;
distance = norm(point - closest);
end

