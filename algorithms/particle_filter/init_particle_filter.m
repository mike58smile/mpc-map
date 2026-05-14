function [public_vars] = init_particle_filter(read_only_vars, public_vars)
%INIT_PARTICLE_FILTER Create a valid, toolbox-free particle cloud.
% A fourth column stores particle weights. The renderer accepts extra
% columns, while the estimator can use the weights for a better pose.

gnss_position = [nan, nan];
if isfield(read_only_vars, 'gnss_position') && numel(read_only_vars.gnss_position) >= 2
	gnss_position = read_only_vars.gnss_position(1:2);
end

particle_count = min(1000, read_only_vars.max_particles);
particles = zeros(particle_count, 4);

seeded_count = 0;
if ~all(isfinite(gnss_position)) && isfield(read_only_vars, 'lidar_distances')
	seeded = lidar_seed_particles(read_only_vars, min(300, particle_count));
	seeded_count = size(seeded, 1);
	particles(1:seeded_count, 1:3) = seeded;
end

for particle_index = seeded_count + 1:particle_count
	particles(particle_index, 1:3) = sample_particle(read_only_vars.map, gnss_position);
end
particles(:, 4) = 1 / particle_count;

public_vars.particles = particles;

end

function particles = lidar_seed_particles(read_only_vars, requested_count)
map = read_only_vars.map;
observed = read_only_vars.lidar_distances(:).';
max_range = hypot(map.limits(3) - map.limits(1), map.limits(4) - map.limits(2));

grid_step = max(0.45, 2 * map.discretization_step);
x_values = (map.limits(1) + 0.25):grid_step:(map.limits(3) - 0.25);
y_values = (map.limits(2) + 0.25):grid_step:(map.limits(4) - 0.25);
headings = 0:(pi / 8):(2 * pi - pi / 8);

max_candidates = numel(x_values) * numel(y_values) * numel(headings);
candidates = inf(max_candidates, 4);
candidate_count = 0;

for x_index = 1:numel(x_values)
	for y_index = 1:numel(y_values)
		x_pos = x_values(x_index);
		y_pos = y_values(y_index);
		if ~is_pose_valid(map, x_pos, y_pos, 0.10)
			continue;
		end
		if is_near_goal_alias(read_only_vars, [x_pos, y_pos])
			continue;
		end

		for heading_index = 1:numel(headings)
			theta = headings(heading_index);
			predicted = compute_lidar_measurement(map, [x_pos, y_pos, theta], read_only_vars.lidar_config);
			error_value = lidar_scan_error(predicted, observed, max_range);
			candidate_count = candidate_count + 1;
			candidates(candidate_count, :) = [error_value, x_pos, y_pos, theta];
		end
	end
end

if candidate_count == 0
	particles = zeros(0, 3);
	return;
end

candidates = sortrows(candidates(1:candidate_count, :), 1);
top_count = min(40, size(candidates, 1));
particles = zeros(requested_count, 3);

for particle_index = 1:requested_count
	candidate = candidates(mod(particle_index - 1, top_count) + 1, :);
	for attempt = 1:25
		x_pos = candidate(2) + 0.10 * randn();
		y_pos = candidate(3) + 0.10 * randn();
		theta = wrap_to_pi(candidate(4) + 0.10 * randn());
		if is_pose_valid(map, x_pos, y_pos, 0.08)
			particles(particle_index, :) = [x_pos, y_pos, theta];
			break;
		end
	end
	if particles(particle_index, 1) == 0 && particles(particle_index, 2) == 0
		particles(particle_index, :) = candidate(2:4);
	end
end
end

function error_value = lidar_scan_error(predicted, observed, max_range)
beam_count = min(numel(predicted), numel(observed));
predicted = predicted(1:beam_count);
observed = observed(1:beam_count);
predicted(~isfinite(predicted)) = max_range;
observed(~isfinite(observed)) = max_range;
error_value = mean(abs(predicted - observed));
end

function near_goal = is_near_goal_alias(read_only_vars, position)
near_goal = false;
if ~isfield(read_only_vars, 'map') || ~isfield(read_only_vars.map, 'goal')
	return;
end

goal_tolerance = 0.5;
if isfield(read_only_vars.map, 'goal_tolerance')
	goal_tolerance = read_only_vars.map.goal_tolerance;
end
near_goal = norm(position - read_only_vars.map.goal(1:2)) <= max(1.2, 2.0 * goal_tolerance);
end

function particle = sample_particle(map, gnss_position)
use_gnss = numel(gnss_position) >= 2 && all(isfinite(gnss_position(1:2)));

for attempt = 1:250
	if use_gnss
		% GNSS is noisy, so sample nearby but keep a global fallback below.
		x_pos = gnss_position(1) + 0.55 * randn();
		y_pos = gnss_position(2) + 0.55 * randn();
	else
		x_pos = map.limits(1) + rand() * (map.limits(3) - map.limits(1));
		y_pos = map.limits(2) + rand() * (map.limits(4) - map.limits(2));
	end

	if is_pose_valid(map, x_pos, y_pos, 0.08)
		particle = [x_pos, y_pos, -pi + 2 * pi * rand()];
		return;
	end

	if attempt == 120
		use_gnss = false;
	end
end

% Extremely defensive fallback for pathological maps.
particle = [mean(map.limits([1, 3])), mean(map.limits([2, 4])), -pi + 2 * pi * rand()];
end

function valid = is_pose_valid(map, x_pos, y_pos, margin)
valid = x_pos > map.limits(1) + margin && x_pos < map.limits(3) - margin && ...
		y_pos > map.limits(2) + margin && y_pos < map.limits(4) - margin;
if ~valid
	return;
end

for wall_index = 1:size(map.walls, 1)
	wall_start = map.walls(wall_index, 1:2);
	wall_end = map.walls(wall_index, 3:4);
	if point_segment_distance([x_pos, y_pos], wall_start, wall_end) < margin
		valid = false;
		return;
	end
end
end

function distance = point_segment_distance(point, segment_start, segment_end)
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

function angle = wrap_to_pi(angle)
angle = mod(angle + pi, 2 * pi) - pi;
end

