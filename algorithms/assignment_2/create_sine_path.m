function [path] = create_sine_path(start_point, end_point, cycles, amplitude_scale)
%CREATE_SINE_PATH Creates a sinusoidal waypoint path between two points.

n_points = 80;
t = linspace(0, 1, n_points)';

d = end_point - start_point;
L = norm(d, 2);

if L < 1e-9
    path = repmat(start_point, n_points, 1);
    return;
end

dir = d / L;
normal = [-dir(2), dir(1)];

amplitude = amplitude_scale * L;

base = start_point + t .* d;
offset = amplitude * sin(2 * pi * cycles * t);

path = base + offset .* normal;

end
