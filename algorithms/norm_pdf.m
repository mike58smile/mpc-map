function [y] = norm_pdf(x, mu, sigma)
%NORM_PDF Normal distribution probability density function.

sigma_safe = max(sigma, eps);
y = (1 ./ (sigma_safe * sqrt(2 * pi))) .* exp(-0.5 * ((x - mu) ./ sigma_safe) .^ 2);

end
