function [new_mu, new_sigma] = kf_correct(mu, sigma, z, kf)
%KF_CORRECT Linear Kalman correction using a GNSS x/y measurement.

if isempty(mu) || isempty(sigma) || isempty(z) || numel(z) < 2 || any(~isfinite(z(1:2)))
    % GNSS is denied or invalid; keep the prediction unchanged.
    new_mu = mu;
    new_sigma = sigma;
    return;
end

C = kf.C;
Q = kf.Q;

S = C * sigma * C.' + Q;
K = sigma * C.' / S;

% Innovation is measured position minus predicted measured position.
innovation = z(1:2) - C * mu;
new_mu = mu + K * innovation;

I = eye(size(sigma, 1));
new_sigma = (I - K * C) * sigma;

new_mu(3) = atan2(sin(new_mu(3)), cos(new_mu(3)));

end
