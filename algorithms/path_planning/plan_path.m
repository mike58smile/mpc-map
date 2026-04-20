function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here

if read_only_vars.counter == 1 || isempty(public_vars.path)
    % Task 1: hand-crafted route on outdoor_1 from [2, 2] to [16, 2].
    waypoints = [
        2.0, 2.0;
        2.0, 4.8;
        7.0, 7.5;
        12.5, 7.5;
        14.7, 3.8;
        16.0, 2.0
    ];

    segment_step = 0.25;
    path = waypoints(1, :);

    for i = 1:size(waypoints, 1)-1
        p0 = waypoints(i, :);
        p1 = waypoints(i+1, :);
        d = norm(p1 - p0, 2);
        n = max(2, ceil(d / segment_step) + 1);

        x = linspace(p0(1), p1(1), n)';
        y = linspace(p0(2), p1(2), n)';
        segment = [x, y];

        % Skip the first sample to prevent duplicates between segments.
        path = [path; segment(2:end, :)]; %#ok<AGROW>
    end
else
    path = public_vars.path;

end
