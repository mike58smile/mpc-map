% Local simulator defaults used when running main.m manually.
% The project algorithm must still work if the evaluator replaces these
% values with arbitrary valid start poses and maps.
start_position = [1, 2, pi/2]; % (x, y, theta)

% Default map for local debugging. Path planning and localization code do
% not assume this specific map; they read geometry from read_only_vars.map.
map_name = 'maps/indoor_2.txt';
