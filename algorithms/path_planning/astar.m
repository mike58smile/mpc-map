function [path] = astar(read_only_vars, public_vars)
%ASTAR Summary of this function goes here

path = [];
if ~isfield(read_only_vars, 'discrete_map') || ~isfield(read_only_vars.discrete_map, 'map')
	return;
end

occ = read_only_vars.discrete_map.map ~= 0;
[ny, nx] = size(occ);
limits = read_only_vars.map.limits;
step = read_only_vars.map.discretization_step;
if isempty(step) || ~isfinite(step) || step <= 0
	step = 1;
end

start_xy = [limits(1), limits(2)];
if isfield(public_vars, 'estimated_pose') && numel(public_vars.estimated_pose) >= 2 && all(isfinite(public_vars.estimated_pose(1:2)))
	start_xy = public_vars.estimated_pose(1:2);
elseif isfield(public_vars, 'mu') && numel(public_vars.mu) >= 2 && all(isfinite(public_vars.mu(1:2)))
	start_xy = public_vars.mu(1:2).';
end
goal_xy = read_only_vars.map.goal(1:2);

sx = clamp_index(round((start_xy(1) - limits(1)) / step) + 1, nx);
sy = clamp_index(round((start_xy(2) - limits(2)) / step) + 1, ny);
gx = clamp_index(round((goal_xy(1) - limits(1)) / step) + 1, nx);
gy = clamp_index(round((goal_xy(2) - limits(2)) / step) + 1, ny);

[sx, sy] = nearest_free(occ, sx, sy);
[gx, gy] = nearest_free(occ, gx, gy);

start_idx = sub2ind([ny, nx], sy, sx);
goal_idx = sub2ind([ny, nx], gy, gx);
num_nodes = nx * ny;

g_score = inf(num_nodes, 1);
f_score = inf(num_nodes, 1);
parent = zeros(num_nodes, 1);
open = false(num_nodes, 1);
closed = false(num_nodes, 1);

g_score(start_idx) = 0;
f_score(start_idx) = heuristic([sx, sy], [gx, gy], step);
open(start_idx) = true;

neighbor_d = [ ...
	-1, -1, sqrt(2) * step; ...
	-1,  0, step; ...
	-1,  1, sqrt(2) * step; ...
	 0, -1, step; ...
	 0,  1, step; ...
	 1, -1, sqrt(2) * step; ...
	 1,  0, step; ...
	 1,  1, sqrt(2) * step];

found = false;
while any(open)
	candidates = find(open);
	[~, rel] = min(f_score(candidates));
	current = candidates(rel);

	if current == goal_idx
		found = true;
		break;
	end

	open(current) = false;
	closed(current) = true;

	[cy, cx] = ind2sub([ny, nx], current);
	for k = 1:size(neighbor_d, 1)
		nx_i = cx + neighbor_d(k, 1);
		ny_i = cy + neighbor_d(k, 2);
		if nx_i < 1 || nx_i > nx || ny_i < 1 || ny_i > ny
			continue;
		end
		if occ(ny_i, nx_i)
			continue;
		end

		nidx = sub2ind([ny, nx], ny_i, nx_i);
		if closed(nidx)
			continue;
		end

		tentative = g_score(current) + neighbor_d(k, 3);
		if ~open(nidx) || tentative < g_score(nidx)
			parent(nidx) = current;
			g_score(nidx) = tentative;
			f_score(nidx) = tentative + heuristic([nx_i, ny_i], [gx, gy], step);
			open(nidx) = true;
		end
	end
end

if ~found
	path = [];
	return;
end

idx_path = goal_idx;
while idx_path(1) ~= start_idx
	p = parent(idx_path(1));
	if p == 0
		path = [];
		return;
	end
	idx_path = [p; idx_path]; %#ok<AGROW>
end

path = zeros(numel(idx_path), 2);
for i = 1:numel(idx_path)
	[py, px] = ind2sub([ny, nx], idx_path(i));
	path(i, :) = [limits(1) + (px - 1) * step, limits(2) + (py - 1) * step];
end

end

function idx = clamp_index(idx, max_val)
idx = max(1, min(max_val, idx));
end

function h = heuristic(a, b, step)
h = hypot((a(1) - b(1)) * step, (a(2) - b(2)) * step);
end

function [fx, fy] = nearest_free(occ, x, y)
if ~occ(y, x)
	fx = x;
	fy = y;
	return;
end

[ys, xs] = find(~occ);
if isempty(xs)
	fx = x;
	fy = y;
	return;
end

[~, idx] = min((xs - x).^2 + (ys - y).^2);
fx = xs(idx);
fy = ys(idx);
end

