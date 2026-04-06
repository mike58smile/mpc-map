function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
          
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);

end

% % Week 2 / Task 2: collect static LiDAR and GNSS data and evaluate noise.
% public_vars = task2_sensor_uncertainty(read_only_vars, public_vars);

% % Week 2 / Task 3: compute covariance matrices from collected sensor data.
% public_vars = task3_covariance_matrix(public_vars);

% % Week 2 / Task 4: generate Gaussian pdfs from selected sensor sigmas.
% public_vars = task4_normal_distribution(public_vars);

% 9. Update particle filter
public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);


end

