function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here

path = astar(read_only_vars, public_vars);
if ~isempty(path)
	path = smooth_path(path);
end

