function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Main student hook called once per simulator iteration.
% The simulator owns read_only_vars; this function updates public_vars only:
% filters, estimated pose, planned path, and wheel command.

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)

    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);

end

% 9. Update particle filter
% The particle filter supplies global pose hypotheses from lidar. It is the
% main absolute localization source when GNSS is denied.
public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
% The EKF predicts from wheel commands and corrects x/y whenever GNSS exists.
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.gnss_available = isfield(read_only_vars, 'gnss_position') && ...
    numel(read_only_vars.gnss_position) >= 2 && all(isfinite(read_only_vars.gnss_position(1:2)));
public_vars.map_goal = read_only_vars.map.goal(1:2);
public_vars.goal_tolerance = read_only_vars.map.goal_tolerance;
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)

% 12. Path planning. A* is expensive, so reuse the path while the robot is
% still close to it and only replan when the estimate drifts away.
if should_replan_path(read_only_vars, public_vars)
    new_path = plan_path(read_only_vars, public_vars);
    if ~isempty(new_path) || isempty(public_vars.path)
        public_vars.path = new_path;
    end
end

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);



end

function replan = should_replan_path(read_only_vars, public_vars)
%SHOULD_REPLAN_PATH Keep A* from running every frame.
% Replanning is triggered when no path exists, the estimate drifts away from
% the path, the goal changes, or a periodic recovery check is due.

replan = false;

counter = 1;
if isfield(read_only_vars, 'counter')
    counter = read_only_vars.counter;
end

if ~isfield(public_vars, 'path') || isempty(public_vars.path)
    % If localization is still settling, try planning occasionally instead
    % of paying the full A* cost on every iteration.
    replan = counter <= 2 || mod(counter, 12) == 0;
    return;
end

if ~isfield(public_vars, 'estimated_pose') || numel(public_vars.estimated_pose) < 2 || ...
        any(~isfinite(public_vars.estimated_pose(1:2)))
    return;
end

goal_xy = read_only_vars.map.goal(1:2);
if norm(public_vars.path(end, :) - goal_xy) > read_only_vars.map.goal_tolerance
    replan = true;
    return;
end

step = read_only_vars.map.discretization_step;
max_path_distance = max(0.9, 4 * step);
distances = sqrt(sum((public_vars.path - public_vars.estimated_pose(1:2)) .^ 2, 2));
if min(distances) > max_path_distance
    % A large lateral distance usually means localization jumped to a new
    % mode or the robot left the old route, so the path must be rebuilt.
    replan = true;
    return;
end

% Periodic cheap check lets the path recover after large localization shifts.
replan = mod(counter, 25) == 0;
end

