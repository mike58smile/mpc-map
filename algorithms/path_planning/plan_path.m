function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Build a safe path from the current estimate to the goal.

if can_reuse_path(read_only_vars, public_vars)
	% Reusing the previous path avoids expensive A* calls on every simulator
	% iteration. student_workspace decides when a full replan is necessary.
	path = public_vars.path;
	return;
end

% A* produces a grid path; smooth_path then removes unnecessary waypoints
% only when the straight shortcut keeps the requested wall clearance.
path = astar(read_only_vars, public_vars);
if ~isempty(path)
	path = smooth_path(path, read_only_vars);
end

end

function reusable = can_reuse_path(read_only_vars, public_vars)
%CAN_REUSE_PATH True when the current estimate is still close to the path.

reusable = false;
if ~isfield(public_vars, 'path') || isempty(public_vars.path) || ...
		~isfield(public_vars, 'estimated_pose') || numel(public_vars.estimated_pose) < 2 || ...
		any(~isfinite(public_vars.estimated_pose(1:2)))
	return;
end

goal_xy = read_only_vars.map.goal(1:2);
if norm(public_vars.path(end, :) - goal_xy) > read_only_vars.map.goal_tolerance
	return;
end

step = read_only_vars.map.discretization_step;
max_path_distance = max(0.9, 4 * step);
distances = sqrt(sum((public_vars.path - public_vars.estimated_pose(1:2)) .^ 2, 2));
reusable = min(distances) <= max_path_distance;
end

