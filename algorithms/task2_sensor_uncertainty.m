function [public_vars] = task2_sensor_uncertainty(read_only_vars, public_vars)
%TASK2_SENSOR_UNCERTAINTY Collects sensor data and plots Task 2 histograms.

% Set up a structure to store Task 2 data if it doesn't exist yet
% This structure will accumulate LiDAR and GNSS samples until we have enough to compute statistics.
if ~isfield(public_vars, 'task2')
    public_vars.task2.required_samples = 150;
    public_vars.task2.lidar_samples = zeros(0, length(read_only_vars.lidar_config));
    public_vars.task2.gnss_samples = zeros(0, 2);
    public_vars.task2.done = false;
end

if public_vars.task2.done
    return;
end

public_vars.task2.lidar_samples(end + 1, :) = read_only_vars.lidar_distances;


% if all(isfinite(read_only_vars.gnss_position))
    public_vars.task2.gnss_samples(end + 1, :) = read_only_vars.gnss_position;
% end


lidar_count = size(public_vars.task2.lidar_samples, 1);
gnss_count = size(public_vars.task2.gnss_samples, 1);

% We need at least the required number of samples for both sensors to compute the statistics and plot the histograms.
if lidar_count < public_vars.task2.required_samples || gnss_count < public_vars.task2.required_samples
    return;
end

% Compute standard deviations (sigma) for LiDAR and GNSS measurements
% w = 0 - Bessel's correction by replacing the denominator n with n-1
%   Reducing the denominator slightly increases the final variance value
%   creating an unbiased estimate that more accurately reflects the parent population's true diversity.
% dim = 1 - compute std across rows (i.e., for each LiDAR channel and for GNSS x/y separately)
sigma_lidar = std(public_vars.task2.lidar_samples, 0, 1);
sigma_gnss = std(public_vars.task2.gnss_samples, 0, 1);

public_vars.task2.sigma_lidar = sigma_lidar;
public_vars.task2.sigma_gnss = sigma_gnss;
public_vars.task2.done = true;

figure(20);
clf; % Clear the current figure to prepare for new plots
% Create a tiled layout for the 8 LiDAR channels in 2 rows and 4 columns
tiledlayout(2, 4, 'Padding', 'compact', 'TileSpacing', 'compact'); 
for i = 1:8
    nexttile; % Move to the next tile in the layout for each channel
    % Plot histohram for the i-th LiDAR channel using 20 bins
    histogram(public_vars.task2.lidar_samples(:, i), 20);
    title(['LiDAR channel ', num2str(i)]);
    xlabel('Distance [m]');
    ylabel('Count');
end
sgtitle('Task 2: LiDAR Measurement Histograms'); % Add a title for the entire figure

figure(21);
clf; % Clear the current figure to prepare for new plots
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
nexttile;
histogram(public_vars.task2.gnss_samples(:, 1), 20);
title('GNSS X');
xlabel('Position X [m]');
ylabel('Count');

nexttile;
histogram(public_vars.task2.gnss_samples(:, 2), 20);
title('GNSS Y');
xlabel('Position Y [m]');
ylabel('Count');
sgtitle('Task 2: GNSS Measurement Histograms');

fprintf('Task 2 completed after %d LiDAR / %d GNSS samples.\n', lidar_count, gnss_count);
fprintf('LiDAR sigma (8 channels): %s\n', mat2str(sigma_lidar, 4));
fprintf('GNSS sigma [x y]: %s\n', mat2str(sigma_gnss, 4));

end
