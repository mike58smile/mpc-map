function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
%COMPUTE_MEASUREMENTS Summary of this function goes here

measurement = zeros(1, length(lidar_config));

origin = pose(1:2);
heading = pose(3);

for i = 1:length(lidar_config)
	direction = heading + lidar_config(i);
	intersections = ray_cast(origin, map.walls, direction);

	if isempty(intersections)
		measurement(i) = inf;
		continue;
	end

	deltas = intersections - origin;
	distances = sqrt(sum(deltas.^2, 2));
	measurement(i) = min(distances);
end

end