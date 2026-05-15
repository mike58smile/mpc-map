function [path] = create_straight_path(start_point, end_point)
%CREATE_STRAIGHT_PATH Creates a straight-line waypoint path.

n_points = 50;
% Evenly sample x and y between endpoints; each row is one waypoint.
x = linspace(start_point(1), end_point(1), n_points)';
y = linspace(start_point(2), end_point(2), n_points)';
path = [x, y];

end
