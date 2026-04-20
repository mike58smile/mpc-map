function [particles] = update_particle_filter(read_only_vars, public_vars)
%UPDATE_PARTICLE_FILTER Summary of this function goes here

particles = public_vars.particles;

% I. Prediction
for i=1:size(particles, 1)
    particles(i,:) = predict_pose(particles(i,:), public_vars.motion_vector, read_only_vars);
end

% II. Correction
measurements = zeros(size(particles,1), length(read_only_vars.lidar_config));
for i=1:size(particles, 1)
    measurements(i,:) = compute_lidar_measurement(read_only_vars.map, particles(i,:), read_only_vars.lidar_config);
end
weights = weight_particles(measurements, read_only_vars.lidar_distances);

% III. Resampling
if ~isempty(weights)
    n_eff = 1 / sum(weights.^2);
else
    n_eff = 0;
end

if n_eff < 0.7 * size(particles, 1)
    particles = resample_particles(particles, weights);
end

% Keep a small exploratory tail to avoid total particle collapse.
N = size(particles, 1);
if N > 0
    n_explore = min(N, max(2, round(0.03 * N)));
    idx = randperm(N, n_explore);

    x_min = read_only_vars.map.limits(1);
    y_min = read_only_vars.map.limits(2);
    x_max = read_only_vars.map.limits(3);
    y_max = read_only_vars.map.limits(4);

    particles(idx, 1) = x_min + (x_max - x_min) * rand(n_explore, 1);
    particles(idx, 2) = y_min + (y_max - y_min) * rand(n_explore, 1);
    particles(idx, 3) = -pi + 2 * pi * rand(n_explore, 1);
end


end

