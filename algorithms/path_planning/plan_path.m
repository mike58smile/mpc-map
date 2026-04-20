function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here

if read_only_vars.counter == 1 || isempty(public_vars.path)
    if isfield(read_only_vars, 'mocap_pose') && numel(read_only_vars.mocap_pose) >= 2
        start_point = read_only_vars.mocap_pose(1:2);
    elseif ~isempty(public_vars.estimated_pose) && numel(public_vars.estimated_pose) >= 2
        start_point = public_vars.estimated_pose(1:2);
    else
        start_point = [0.0, 0.0];
    end

    if isfield(read_only_vars, 'map') && isfield(read_only_vars.map, 'goal') && numel(read_only_vars.map.goal) >= 2
        end_point = read_only_vars.map.goal(1:2);
    else
        end_point = [21.0, 4.0];
    end

    n_points = 50;
    x = linspace(start_point(1), end_point(1), n_points)';
    y = linspace(start_point(2), end_point(2), n_points)';
    path = [x, y];
else
    path = public_vars.path;

end
