function [target] = get_target(estimated_pose, path)
%GET_TARGET Summary of this function goes here

if isempty(path)
	if isempty(estimated_pose)
		target = [0.0, 0.0];
	else
		target = estimated_pose(1:2);
	end
	return;
end

position = estimated_pose(1:2);

distances = vecnorm(path - position, 2, 2);
[~, nearest_idx] = min(distances);

lookahead = 5;
target_idx = min(nearest_idx + lookahead, size(path, 1));

target = path(target_idx, :);

end

