function [path] = create_circular_path(start_point, end_point, n_arcs, height_scale)
%CREATE_CIRCULAR_PATH Creates chained circular arcs between two points.

n_points = 70;

d = end_point - start_point;
% L = Euclidean distance between start and end
L = norm(d, 2);

% n_arcs is rounded and forced to at least 1.
n_arcs = max(1, round(n_arcs));
% height_scale is clamped to [0.0, 1.0]
height_scale = max(0.0, min(1.0, height_scale));

% dir is the unit vector from start to end
dir = d / L;
normal = [-dir(2), dir(1)];

segment_length = L / n_arcs;
radius = 0.5 * segment_length;
amplitude_scale = (radius * height_scale) / max(radius, 1e-9);
s = linspace(0.0, L, n_points)';

path = zeros(n_points, 2);

for i = 1:n_points
    si = s(i);
    seg_idx = min(floor(si / segment_length), n_arcs - 1);
    local_s = si - seg_idx * segment_length;
    local_x = local_s - radius;
    local_y = sqrt(max(radius^2 - local_x^2, 0.0));

    if mod(seg_idx, 2) == 1
        local_y = -local_y;
    end

    base_point = start_point + (si / L) * d;
    path(i, :) = base_point + amplitude_scale * local_y * normal;
end

% Enforce exact start/end points.
path(1, :) = start_point;
path(end, :) = end_point;

end
