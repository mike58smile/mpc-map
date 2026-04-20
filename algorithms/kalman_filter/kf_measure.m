function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf)
%KF_MEASURE Summary of this function goes here

[new_mu, new_sigma] = kf_correct(mu, sigma, z, kf);

end

