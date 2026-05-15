function [path] = astar(read_only_vars, public_vars)
%ASTAR Grid A* with obstacle clearance based on wall geometry.

map = read_only_vars.map;
step = map.discretization_step;
if isempty(step) || ~isfinite(step) || step <= 0
	step = 0.2;
end

x_grid = map.limits(1):step:map.limits(3);
y_grid = map.limits(2):step:map.limits(4);
column_count = numel(x_grid);
row_count = numel(y_grid);

start_xy = choose_start(public_vars, map);
goal_xy = map.goal(1:2);

% Try safer inflated grids first. If a narrow map cannot be solved with the
% larger clearances, progressively fall back to tighter clearances.
clearance_candidates = [max(0.35, 1.6 * step), max(0.28, 1.3 * step), ...
	max(0.22, 1.1 * step), 1e-6];
boundary_candidates = [max(0.16, 0.8 * step), max(0.12, 0.6 * step), ...
	max(0.10, 0.5 * step), 0.0];

for clearance_index = 1:numel(clearance_candidates)
	% Occupancy is built from wall geometry, not from a toolbox image-dilation
	% function, so the planner stays toolbox-free.
	occupied = build_occupancy_grid(x_grid, y_grid, map.walls, map.limits, ...
		clearance_candidates(clearance_index), boundary_candidates(clearance_index));
	path = search_grid(occupied, x_grid, y_grid, start_xy, goal_xy, step, row_count, column_count);
	if ~isempty(path)
		return;
	end
end

% Final fallback: a finer grid can represent tight passages that disappear
% on the default 0.2 m grid. This is used only after normal clearances fail.
fine_step = step / 2;
fine_x_grid = map.limits(1):fine_step:map.limits(3);
fine_y_grid = map.limits(2):fine_step:map.limits(4);
fine_occupied = build_occupancy_grid(fine_x_grid, fine_y_grid, map.walls, map.limits, 1e-6, 0.0);
path = search_grid(fine_occupied, fine_x_grid, fine_y_grid, start_xy, goal_xy, ...
	fine_step, numel(fine_y_grid), numel(fine_x_grid));

end

function path = search_grid(occupied, x_grid, y_grid, start_xy, goal_xy, step, row_count, column_count)
%SEARCH_GRID Standard 8-connected A* over a precomputed occupancy grid.

path = [];

start_column = nearest_index(x_grid, start_xy(1));
start_row = nearest_index(y_grid, start_xy(2));
goal_column = nearest_index(x_grid, goal_xy(1));
goal_row = nearest_index(y_grid, goal_xy(2));

[start_column, start_row] = nearest_free(occupied, start_column, start_row);
[goal_column, goal_row] = nearest_free(occupied, goal_column, goal_row);

start_index = sub2ind([row_count, column_count], start_row, start_column);
goal_index = sub2ind([row_count, column_count], goal_row, goal_column);
node_count = row_count * column_count;

g_score = inf(node_count, 1);
f_score = inf(node_count, 1);
parent = zeros(node_count, 1);
open_set = false(node_count, 1);
closed_set = false(node_count, 1);

g_score(start_index) = 0;
f_score(start_index) = heuristic([start_column, start_row], [goal_column, goal_row], step);
open_set(start_index) = true;

neighbors = [ ...
	-1, -1, sqrt(2) * step; ...
	-1,  0, step; ...
	-1,  1, sqrt(2) * step; ...
	 0, -1, step; ...
	 0,  1, step; ...
	 1, -1, sqrt(2) * step; ...
	 1,  0, step; ...
	 1,  1, sqrt(2) * step];

found = false;
while any(open_set)
	% For these small maps, scanning the open set is simple and sufficiently
	% fast. A heap would be more code without much benefit here.
	candidates = find(open_set);
	[~, relative_index] = min(f_score(candidates));
	current = candidates(relative_index);

	if current == goal_index
		found = true;
		break;
	end

	open_set(current) = false;
	closed_set(current) = true;

	[current_row, current_column] = ind2sub([row_count, column_count], current);
	for neighbor_index = 1:size(neighbors, 1)
		next_column = current_column + neighbors(neighbor_index, 1);
		next_row = current_row + neighbors(neighbor_index, 2);

		if next_column < 1 || next_column > column_count || next_row < 1 || next_row > row_count
			continue;
		end
		if occupied(next_row, next_column)
			continue;
		end

		next_index = sub2ind([row_count, column_count], next_row, next_column);
		if closed_set(next_index)
			continue;
		end

		tentative_score = g_score(current) + neighbors(neighbor_index, 3);
		if ~open_set(next_index) || tentative_score < g_score(next_index)
			parent(next_index) = current;
			g_score(next_index) = tentative_score;
			f_score(next_index) = tentative_score + heuristic([next_column, next_row], [goal_column, goal_row], step);
			open_set(next_index) = true;
		end
	end
end

if ~found
	return;
end

index_path = goal_index;
while index_path(1) ~= start_index
	previous = parent(index_path(1));
	if previous == 0
		path = [];
		return;
	end
	index_path = [previous; index_path]; %#ok<AGROW>
end

path = zeros(numel(index_path), 2);
for path_index = 1:numel(index_path)
	% Convert grid indices back to simulator/world coordinates.
	[path_row, path_column] = ind2sub([row_count, column_count], index_path(path_index));
	path(path_index, :) = [x_grid(path_column), y_grid(path_row)];
end

end

function start_xy = choose_start(public_vars, map)
%CHOOSE_START Prefer the current pose estimate, then EKF, then particle mean.

if isfield(public_vars, 'estimated_pose') && numel(public_vars.estimated_pose) >= 2 && ...
		all(isfinite(public_vars.estimated_pose(1:2)))
	start_xy = public_vars.estimated_pose(1:2);
elseif isfield(public_vars, 'mu') && numel(public_vars.mu) >= 2 && all(isfinite(public_vars.mu(1:2)))
	start_xy = public_vars.mu(1:2).';
elseif isfield(public_vars, 'particles') && ~isempty(public_vars.particles)
	start_xy = mean(public_vars.particles(:, 1:2), 1);
else
	start_xy = [mean(map.limits([1, 3])), mean(map.limits([2, 4]))];
end
end

function index = nearest_index(values, value)
[~, index] = min(abs(values - value));
end

function distance = heuristic(a, b, step)
distance = hypot((a(1) - b(1)) * step, (a(2) - b(2)) * step);
end

function occupied = build_occupancy_grid(x_grid, y_grid, walls, limits, clearance, boundary_clearance)
%BUILD_OCCUPANCY_GRID Mark cells too close to walls or arena borders.

[grid_x, grid_y] = meshgrid(x_grid, y_grid);
occupied = grid_x <= limits(1) + boundary_clearance | grid_x >= limits(3) - boundary_clearance | ...
		   grid_y <= limits(2) + boundary_clearance | grid_y >= limits(4) - boundary_clearance;

for wall_index = 1:size(walls, 1)
	wall_start = walls(wall_index, 1:2);
	wall_end = walls(wall_index, 3:4);
	distance = point_segment_distance_grid(grid_x, grid_y, wall_start, wall_end);
	occupied = occupied | distance <= clearance;
end
end

function distance = point_segment_distance_grid(grid_x, grid_y, segment_start, segment_end)
%POINT_SEGMENT_DISTANCE_GRID Vectorized point-to-wall distance for one wall.

segment = segment_end - segment_start;
denominator = segment(1) ^ 2 + segment(2) ^ 2;
if denominator < eps
	distance = hypot(grid_x - segment_start(1), grid_y - segment_start(2));
	return;
end

projection = ((grid_x - segment_start(1)) * segment(1) + (grid_y - segment_start(2)) * segment(2)) / denominator;
projection = max(0, min(1, projection));
closest_x = segment_start(1) + projection * segment(1);
closest_y = segment_start(2) + projection * segment(2);
distance = hypot(grid_x - closest_x, grid_y - closest_y);
end

function [free_column, free_row] = nearest_free(occupied, column, row)
%NEAREST_FREE Move start/goal off an occupied cell if inflation covers it.

if ~occupied(row, column)
	free_column = column;
	free_row = row;
	return;
end

[free_rows, free_columns] = find(~occupied);
if isempty(free_columns)
	free_column = column;
	free_row = row;
	return;
end

[~, nearest] = min((free_columns - column) .^ 2 + (free_rows - row) .^ 2);
free_column = free_columns(nearest);
free_row = free_rows(nearest);
end

