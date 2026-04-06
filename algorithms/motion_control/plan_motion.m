function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Task 5: open-loop motion without sensor feedback.

% I. Pick navigation target

target = get_target(public_vars.estimated_pose, public_vars.path);

% II. Plan motion command to reach target

c = read_only_vars.counter;

% Differential-drive wheel speeds [vR, vL].
v_fwdL = 0.501;
v_fwdR = 0.5;

v_turn = 0.2;

% Segment lengths (in iterations at Ts = 0.1 s):
% up, right, down, right, up (observer perspective)
n_up_1 = 140;
n_right_1 = 85;
n_down = 140;
n_right_2 = 70;
n_up_2 = 140;
n_turn_90 = 8;
n_turn_90_2 = 7;

i1 = n_up_1;
i2 = i1 + n_turn_90;
i3 = i2 + n_right_1;
i4 = i3 + n_turn_90;
i5 = i4 + n_down;
i6 = i5 + n_turn_90;
i7 = i6 + n_right_2;
i8 = i7 + n_turn_90_2;
i9 = i8 + n_up_2;

if c <= i1
	% 1) Up
	public_vars.motion_vector = [v_fwdR, v_fwdL];
elseif c <= i2
	% Turn right (clockwise): up -> right
	public_vars.motion_vector = [-v_turn, v_turn];
elseif c <= i3
	% 2) Right
	public_vars.motion_vector = [v_fwdR, v_fwdL];
elseif c <= i4
	% Turn right (clockwise): right -> down
	public_vars.motion_vector = [-v_turn, v_turn];
elseif c <= i5
	% 3) Down
	public_vars.motion_vector = [v_fwdR, v_fwdL];
elseif c <= i6
	% Turn left (counter-clockwise): down -> right
	public_vars.motion_vector = [v_turn, -v_turn];
elseif c <= i7
	% 4) Right
	public_vars.motion_vector = [v_fwdR, v_fwdL];
elseif c <= i8
	% Turn left (counter-clockwise): right -> up
	public_vars.motion_vector = [v_turn, -v_turn];
elseif c <= i9
	% 5) Up
	public_vars.motion_vector = [v_fwdR, v_fwdL];
else
	public_vars.motion_vector = [0.0, 0.0];
end


end