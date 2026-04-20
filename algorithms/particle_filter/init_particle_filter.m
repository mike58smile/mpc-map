function [public_vars] = init_particle_filter(read_only_vars, public_vars)
%INIT_PARTICLE_FILTER Summary of this function goes here

N = min(500, read_only_vars.max_particles);

x_min = read_only_vars.map.limits(1);
y_min = read_only_vars.map.limits(2);
x_max = read_only_vars.map.limits(3);
y_max = read_only_vars.map.limits(4);

x = x_min + (x_max - x_min) * rand(N, 1);
y = y_min + (y_max - y_min) * rand(N, 1);
theta = -pi + 2 * pi * rand(N, 1);

public_vars.particles = [x, y, theta];

end

