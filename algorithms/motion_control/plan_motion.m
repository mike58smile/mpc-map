function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Task 3: simple path-following with MoCap pose.

required_samples = 80;
if isfield(public_vars, 'kf') && isfield(public_vars.kf, 'gnss_init_required')
	required_samples = public_vars.kf.gnss_init_required;
end

valid_count = 0;
if isfield(read_only_vars, 'gnss_history') && ~isempty(read_only_vars.gnss_history)
	valid_count = sum(all(isfinite(read_only_vars.gnss_history(:, 1:2)), 2));
elseif isfield(read_only_vars, 'gnss_position') && numel(read_only_vars.gnss_position) >= 2 && all(isfinite(read_only_vars.gnss_position(1:2)))
	valid_count = 1;
end

if valid_count < required_samples
	public_vars.motion_vector = [0.0, 0.0];
	return;
end

% I. Pick navigation target
current_pose = public_vars.estimated_pose;

target = get_target(current_pose, public_vars.path);

% II. Plan motion command to reach target
public_vars.motion_vector = simple_path_following_control( ...
	current_pose, ...
	target, ...
	read_only_vars.agent_drive.interwheel_dist, ...
	read_only_vars.agent_drive.max_vel);

end

function [motion_vector] = simple_path_following_control(current_pose, target, interwheel_dist, max_vel)
%SIMPLE_PATH_FOLLOWING_CONTROL Differential-drive proportional controller.

if isempty(current_pose) || numel(current_pose) < 3 || isempty(target) || numel(target) < 2
	motion_vector = [0.0, 0.0];
	return;
end

position = current_pose(1:2);
theta = current_pose(3);

delta = target - position;
distance = norm(delta, 2);

if distance < 0.05
	motion_vector = [0.0, 0.0];
	return;
end

desired_heading = atan2(delta(2), delta(1));
heading_error = atan2(sin(desired_heading - theta), cos(desired_heading - theta));

k_v = 1.20;   % was 0.45
k_w = 1.4;    % was 1.6 (slightly lower to avoid oscillation at higher speed)

base_v = min(0.35 * max_vel, k_v * distance); % was 0.12 * max_vel
heading_slowdown = max(0.0, cos(heading_error));
v = base_v * heading_slowdown;
omega = k_w * heading_error;

vR = v + 0.5 * interwheel_dist * omega;
vL = v - 0.5 * interwheel_dist * omega;

vR = max(-max_vel, min(max_vel, vR));
vL = max(-max_vel, min(max_vel, vL));

motion_vector = [vR, vL];

end