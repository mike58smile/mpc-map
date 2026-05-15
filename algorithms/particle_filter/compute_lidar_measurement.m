function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
%COMPUTE_LIDAR_MEASUREMENT Simulate lidar beams from a candidate pose.

measurement = zeros(1, length(lidar_config));

origin = pose(1:2);
heading = pose(3);

for i = 1:length(lidar_config)
	direction = heading + lidar_config(i);
	% ray_cast returns all wall intersections for one beam; the lidar reports
	% the nearest hit distance, or Inf if no wall is hit.
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