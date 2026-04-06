function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Task 3: simple path-following with MoCap pose.

% I. Pick navigation target
if isfield(read_only_vars, 'mocap_pose') && numel(read_only_vars.mocap_pose) >= 3
	current_pose = read_only_vars.mocap_pose;
else
	current_pose = public_vars.estimated_pose;
end

target = get_target(current_pose, public_vars.path);

% II. Plan motion command to reach target
public_vars.motion_vector = task3_motion_control( ...
	current_pose, ...
	target, ...
	read_only_vars.agent_drive.interwheel_dist, ...
	read_only_vars.agent_drive.max_vel);

end