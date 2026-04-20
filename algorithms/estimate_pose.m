function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Summary of this function goes here

if isfield(public_vars, 'mu') && numel(public_vars.mu) == 3 && all(isfinite(public_vars.mu))
	estimated_pose = public_vars.mu;
	return;
end

if isfield(public_vars, 'particles') && ~isempty(public_vars.particles) && size(public_vars.particles, 2) >= 3
	mean_theta = atan2(mean(sin(public_vars.particles(:,3))), mean(cos(public_vars.particles(:,3))));
	estimated_pose = [mean(public_vars.particles(:,1:2), 1), mean_theta];
	return;
end

estimated_pose = [nan, nan, nan];

end

