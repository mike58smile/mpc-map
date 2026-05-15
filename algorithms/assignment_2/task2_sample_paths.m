function [path] = task2_sample_paths(path_name, start_point, end_point, path_params)
%TASK2_SAMPLE_PATHS Returns one of three test paths for Week 3 Task 2.

start_point = start_point(1:2);
end_point = end_point(1:2);

sine_cycles = path_params.sine_cycles;
sine_amplitude_scale = path_params.sine_amplitude_scale;
circular_arcs = path_params.circular_arcs;
circular_height_scale = path_params.circular_height_scale;

% Dispatch by a human-readable path name used in the assignment GUI/script.
switch lower(path_name)
    case {'1', 'straight', 'line'}
        path = create_straight_path(start_point, end_point);

    case {'2', 'arc', 'circular'}
        path = create_circular_path(start_point, end_point, circular_arcs, circular_height_scale);

    case {'3', 'sine', 'sinusoid'}
        path = create_sine_path(start_point, end_point, sine_cycles, sine_amplitude_scale);

    otherwise
        error('Unknown path_name. Use straight, arc, or sine.');
end

end
