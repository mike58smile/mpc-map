function [target] = get_target(estimated_pose, path)
%GET_TARGET Pick a lookahead point on the planned path.

if isempty(path)
	if isempty(estimated_pose) || numel(estimated_pose) < 2
		target = [0.0, 0.0];
	else
		target = estimated_pose(1:2);
	end
	return;
end

if isempty(estimated_pose) || numel(estimated_pose) < 2 || any(~isfinite(estimated_pose(1:2)))
	target = path(1, :);
	return;
end

position = estimated_pose(1:2);
delta = path - position;
distances = sqrt(sum(delta .^ 2, 2));
[~, nearest_index] = min(distances);

lookahead_distance = 0.35;
travel = 0;
target_index = nearest_index;
while target_index < size(path, 1) && travel < lookahead_distance
	step_distance = norm(path(target_index + 1, :) - path(target_index, :));
	travel = travel + step_distance;
	target_index = target_index + 1;
end

target = path(target_index, :);

end

