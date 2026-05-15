function [y] = norm_pdf(x, mu, sigma)
%NORM_PDF Normal distribution probability density function.

% Guard sigma so a zero measured variance cannot create division by zero.
sigma_safe = max(sigma, eps);

% Element-wise Gaussian pdf, kept local to avoid toolbox dependencies.
y = (1 ./ (sigma_safe * sqrt(2 * pi))) .* exp(-0.5 * ((x - mu) ./ sigma_safe) .^ 2);

end
