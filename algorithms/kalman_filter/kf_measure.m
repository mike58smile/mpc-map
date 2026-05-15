function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf)
%KF_MEASURE Compatibility wrapper around the Kalman correction step.

[new_mu, new_sigma] = kf_correct(mu, sigma, z, kf);

end

