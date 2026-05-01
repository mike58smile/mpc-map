function [new_path] = smooth_path(old_path)
%SMOOTH_PATH Summary of this function goes here

new_path = old_path;
if size(old_path, 1) <= 2
	return;
end

alpha = 0.5;
beta = 0.25;
% Try also: alpha = 0.25; beta = 0.5;

max_iter = 80;
tol = 1e-4;

Y = old_path;
for k = 1:max_iter
	Y_prev = Y;
	for i = 2:size(Y,1)-1
		Y(i,:) = Y_prev(i,:) + alpha * (old_path(i,:) - Y_prev(i,:)) + ...
			beta * (Y_prev(i-1,:) + Y_prev(i+1,:) - 2 * Y_prev(i,:));
	end
	if max(abs(Y(:) - Y_prev(:))) < tol
		break;
	end
end

new_path = Y;

end

