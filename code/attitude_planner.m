function [rot, omega] = attitude_planner(desired_state, params)

% Input parameters
%
%   desired_state: The desired states are:
%   desired_state.pos = [x; y; z], 
%   desired_state.vel = [x_dot; y_dot; z_dot],
%   desired_state.rot = [phi; theta; psi], 
%   desired_state.omega = [phidot; thetadot; psidot]
%   desired_state.acc = [xdotdot; ydotdot; zdotdot];
%
%   params: Quadcopter parameters
%
% Output parameters
%
%   rot: will be stored as desired_state.rot = [phi; theta; psi], 
%
%   omega: will be stored as desired_state.omega = [phidot; thetadot; psidot]
%
%************  ATTITUDE PLANNER ************************

psi = desired_state.rot(3);
psi_dot = desired_state.omega(3);

desired_state.rot(1:2) = (1/(params.gravity))*[sin(psi) -cos(psi); cos(psi) sin(psi)]*[desired_state.acc(1); desired_state.acc(2)];

desired_state.omega(1:2) = (1/(params.gravity))*[cos(psi) sin(psi); -sin(psi) cos(psi)]*[desired_state.acc(1)*psi_dot; desired_state.acc(2)*psi_dot];

rot = desired_state.rot(1:3);
omega = desired_state.omega(1:3);

end

