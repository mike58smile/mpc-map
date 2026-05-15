function [motion_vector] = task3_motion_control(current_pose, target, interwheel_dist, max_vel)
%TASK3_MOTION_CONTROL Simple proportional path-following controller.

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

% Proportional controller in linear/angular velocity space. The linear speed
% is capped so the robot does not overshoot sparse path waypoints.
k_v = 0.7;
k_w = 2.0;

v = min(0.2 * max_vel, k_v * distance);
omega = k_w * heading_error;

% Convert unicycle command (v, omega) into differential wheel speeds.
vR = v + 0.5 * interwheel_dist * omega;
vL = v - 0.5 * interwheel_dist * omega;

vR = max(-max_vel, min(max_vel, vR));
vL = max(-max_vel, min(max_vel, vL));

motion_vector = [vR, vL];

end
