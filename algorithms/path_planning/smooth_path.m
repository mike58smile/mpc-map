function [new_path] = smooth_path(old_path, read_only_vars)
%SMOOTH_PATH Shortcut an A* path while preserving obstacle clearance.

new_path = old_path;
if size(old_path, 1) <= 2
	return;
end

if nargin < 2 || ~isfield(read_only_vars, 'map')
	new_path = numeric_smooth(old_path);
	return;
end

map = read_only_vars.map;
step = map.discretization_step;
% Shortcut segments must be farther from walls than the raw A* clearance,
% otherwise smoothing could cut across obstacle corners.
clearance = max(0.55, 2.5 * step);

shortcut = old_path(1, :);
anchor_index = 1;
while anchor_index < size(old_path, 1)
	next_index = anchor_index + 1;
	% Greedily connect the current anchor to the farthest later waypoint that
	% remains collision-free with the chosen clearance.
	for candidate_index = size(old_path, 1):-1:(anchor_index + 1)
		if segment_is_clear(old_path(anchor_index, :), old_path(candidate_index, :), map, clearance)
			next_index = candidate_index;
			break;
		end
	end
	shortcut = [shortcut; old_path(next_index, :)]; %#ok<AGROW>
	anchor_index = next_index;
end

new_path = shortcut;
end

function clear = segment_is_clear(start_point, end_point, map, clearance)
%SEGMENT_IS_CLEAR Sample a straight segment and test clearance to every wall.

segment_length = norm(end_point - start_point);
sample_count = max(2, ceil(segment_length / max(0.05, map.discretization_step / 2)));
clear = true;

for sample_index = 0:sample_count
	ratio = sample_index / sample_count;
	point = start_point + ratio * (end_point - start_point);
	if point(1) <= map.limits(1) + 0.08 || point(1) >= map.limits(3) - 0.08 || ...
			point(2) <= map.limits(2) + 0.08 || point(2) >= map.limits(4) - 0.08
		clear = false;
		return;
	end

	for wall_index = 1:size(map.walls, 1)
		if point_segment_distance(point, map.walls(wall_index, 1:2), map.walls(wall_index, 3:4)) < clearance
			clear = false;
			return;
		end
	end
end
end

function distance = point_segment_distance(point, segment_start, segment_end)
%POINT_SEGMENT_DISTANCE Distance from one sampled point to one wall segment.

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

function smoothed = numeric_smooth(path)
%NUMERIC_SMOOTH Fallback smoother when map geometry is unavailable.

smoothed = path;
alpha = 0.5;
beta = 0.25;
for iteration = 1:80
	previous = smoothed;
	for path_index = 2:size(smoothed, 1) - 1
		smoothed(path_index, :) = previous(path_index, :) + ...
			alpha * (path(path_index, :) - previous(path_index, :)) + ...
			beta * (previous(path_index - 1, :) + previous(path_index + 1, :) - 2 * previous(path_index, :));
	end
	if max(abs(smoothed(:) - previous(:))) < 1e-4
		break;
	end
end
end

