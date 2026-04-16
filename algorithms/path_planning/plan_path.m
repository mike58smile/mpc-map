function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here

planning_required = 0;

if planning_required
    
    path = astar(read_only_vars, public_vars);
    
    path = smooth_path(path);
    
else
    % Week 3 / Task 2: define and visualize sample paths.
    selected_path = 'sine'; % change to: 'straight', 'circular', or 'sine'

    % Curve parameters for creating smaller/more frequent curves.
    path_params.sine_cycles = 10.0;
    path_params.sine_amplitude_scale = 0.05;
    path_params.circular_arcs = 12;
    path_params.circular_height_scale = 0.80;

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

        path = task2_sample_paths(selected_path, start_point, end_point, path_params);
    else
        path = public_vars.path;
    end

end
end
