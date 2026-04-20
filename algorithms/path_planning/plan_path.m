function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here

if read_only_vars.counter == 1 || isempty(public_vars.path)
    if isfield(read_only_vars, 'map') && isfield(read_only_vars.map, 'goal') && numel(read_only_vars.map.goal) >= 2
        end_point = read_only_vars.map.goal(1:2);
    else
        end_point = [16.0, 2.0];
    end

    % Task 1: simple manually designed trajectory for outdoor_1 map.
    waypoints = [
        2.0, 2.0;
        3.0, 6.5;
        13.0, 7.2;
        end_point(1), end_point(2)
    ];

    points_per_segment = 20;
    total_points = points_per_segment + (size(waypoints, 1) - 2) * (points_per_segment - 1);
    path = zeros(total_points, 2);
    write_idx = 1;
    for i = 1 : size(waypoints, 1) - 1
        t = linspace(0, 1, points_per_segment)';
        segment = (1 - t) .* waypoints(i, :) + t .* waypoints(i + 1, :);
        if i > 1
            segment = segment(2:end, :);
        end
        segment_len = size(segment, 1);
        path(write_idx:write_idx + segment_len - 1, :) = segment;
        write_idx = write_idx + segment_len;
    end
else
    path = public_vars.path;

end
