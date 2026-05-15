function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Follow the planned path with a differential-drive controller.

current_pose = public_vars.estimated_pose;
gnss_available = isfield(public_vars, 'gnss_available') && public_vars.gnss_available;
if ~gnss_available && isfield(read_only_vars, 'counter') && read_only_vars.counter < 35
	% Rotating in place is safe and gives the particle filter several lidar
	% views before the robot commits to a corridor or wall opening.
	public_vars.motion_vector = [0.22, -0.22];
	return;
end

if isempty(current_pose) || numel(current_pose) < 3 || any(~isfinite(current_pose(1:3))) || isempty(public_vars.path)
	% Spin slowly in place so lidar can recover localization without drifting.
	public_vars.motion_vector = [0.18, -0.18];
	return;
end

if ~gnss_available && norm(current_pose(1:2) - read_only_vars.map.goal(1:2)) < read_only_vars.map.goal_tolerance
	% If this function is running, the simulator's true goal check has already
	% failed. A no-GNSS estimate exactly at the goal is therefore a localization
	% alias; keep moving safely so the lidar history can disambiguate it.
	public_vars.motion_vector = local_exploration_motion(read_only_vars);
	return;
end

target = get_target(current_pose, public_vars.path);
public_vars.motion_vector = simple_path_following_control(current_pose, target, read_only_vars);

end

function [motion_vector] = simple_path_following_control(current_pose, target, read_only_vars)
%SIMPLE_PATH_FOLLOWING_CONTROL Convert target point into wheel velocities.

if isempty(target) || numel(target) < 2 || any(~isfinite(target(1:2)))
	motion_vector = [0.0, 0.0];
	return;
end

position = current_pose(1:2);
heading = current_pose(3);
delta = target(1:2) - position;
distance_to_target = norm(delta);

if distance_to_target < 0.08
	motion_vector = [0.0, 0.0];
	return;
end

desired_heading = atan2(delta(2), delta(1));
heading_error = wrap_to_pi(desired_heading - heading);
max_velocity = read_only_vars.agent_drive.max_vel;

% Turn first when the target is far off-axis; otherwise advance smoothly.
if abs(heading_error) > 0.55
	linear_velocity = 0.0;
else
	linear_velocity = min(0.85 * max_velocity, 1.05 * distance_to_target);
	linear_velocity = linear_velocity * max(0.20, cos(heading_error));
end
angular_velocity = 3.0 * heading_error;

% Lidar is used as a final reactive safety layer on top of the planned path.
[linear_velocity, angular_velocity] = avoid_close_front_obstacle(linear_velocity, angular_velocity, read_only_vars);

wheel_base = read_only_vars.agent_drive.interwheel_dist;
right_velocity = linear_velocity + 0.5 * wheel_base * angular_velocity;
left_velocity = linear_velocity - 0.5 * wheel_base * angular_velocity;

scale = max(1.0, max(abs([right_velocity, left_velocity])) / max_velocity);
% Scale both wheels together to respect the differential-drive velocity limit
% without changing the commanded curvature.
right_velocity = right_velocity / scale;
left_velocity = left_velocity / scale;

motion_vector = [right_velocity, left_velocity];
end

function [linear_velocity, angular_velocity] = avoid_close_front_obstacle(linear_velocity, angular_velocity, read_only_vars)
%AVOID_CLOSE_FRONT_OBSTACLE Slow down and turn away from close frontal walls.

if ~isfield(read_only_vars, 'lidar_distances') || isempty(read_only_vars.lidar_distances)
	return;
end

angles = wrap_to_pi(read_only_vars.lidar_config);
front = abs(angles) <= pi / 4;
left = angles > 0 & angles <= pi;
right = angles < 0 & angles >= -pi;
front_distances = read_only_vars.lidar_distances(front);
front_distances = front_distances(isfinite(front_distances));

if isempty(front_distances)
	return;
end

front_distance = min(front_distances);
if front_distance > 0.42
	return;
end

linear_velocity = min(linear_velocity, 0.04);
left_clearance = side_clearance(read_only_vars.lidar_distances(left));
right_clearance = side_clearance(read_only_vars.lidar_distances(right));
if left_clearance >= right_clearance
	angular_velocity = max(angular_velocity, 1.8);
else
	angular_velocity = min(angular_velocity, -1.8);
end
end

function clearance = side_clearance(distances)
finite_distances = distances(isfinite(distances));
if isempty(finite_distances)
	clearance = inf;
else
	clearance = min(finite_distances);
end
end

function motion_vector = local_exploration_motion(read_only_vars)
%LOCAL_EXPLORATION_MOTION Safe motion when localization is clearly aliased.

angles = wrap_to_pi(read_only_vars.lidar_config);
distances = read_only_vars.lidar_distances;
front_clearance = sector_clearance(distances, abs(angles) <= pi / 5);
left_clearance = sector_clearance(distances, angles > 0 & angles <= pi);
right_clearance = sector_clearance(distances, angles < 0 & angles >= -pi);

if front_clearance > 0.55
	linear_velocity = 0.32;
	angular_velocity = 0.35 * sign(right_clearance - left_clearance);
else
	linear_velocity = 0.0;
	if left_clearance >= right_clearance
		angular_velocity = 2.0;
	else
		angular_velocity = -2.0;
	end
end

wheel_base = read_only_vars.agent_drive.interwheel_dist;
right_velocity = linear_velocity + 0.5 * wheel_base * angular_velocity;
left_velocity = linear_velocity - 0.5 * wheel_base * angular_velocity;
max_velocity = read_only_vars.agent_drive.max_vel;
scale = max(1.0, max(abs([right_velocity, left_velocity])) / max_velocity);
motion_vector = [right_velocity, left_velocity] / scale;
end

function clearance = sector_clearance(distances, mask)
%SECTOR_CLEARANCE Minimum finite lidar distance inside an angular sector.

sector_distances = distances(mask);
sector_distances = sector_distances(isfinite(sector_distances));
if isempty(sector_distances)
	clearance = inf;
else
	clearance = min(sector_distances);
end
end

function angle = wrap_to_pi(angle)
%WRAP_TO_PI Local toolbox-free angle wrapping helper.

angle = mod(angle + pi, 2 * pi) - pi;
end
