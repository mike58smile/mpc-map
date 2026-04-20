function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Summary of this function goes here

if isfield(public_vars, 'mu') && numel(public_vars.mu) >= 3 && all(isfinite(public_vars.mu(1:3)))
	estimated_pose = public_vars.mu(1:3).';
else
	estimated_pose = nan(1,3);
end

end

