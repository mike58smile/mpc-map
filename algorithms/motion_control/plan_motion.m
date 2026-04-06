function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Task 5: open-loop motion without sensor feedback.

% I. Pick navigation target

target = get_target(public_vars.estimated_pose, public_vars.path);

% II. Plan motion command to reach target

public_vars.motion_vector = [0.5, 0.5]; % [vR, vL]


end