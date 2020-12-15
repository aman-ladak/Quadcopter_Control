function [state_dot] = dynamics(params, state, F, M, rpm_motor_dot)
% Input parameters
% 
%   state: current state, will be using ode45 to update
%
%   F, M: actual force and moment from motor model
%
%   rpm_motor_dot: actual change in RPM from motor model
% 
%   params: Quadcopter parameters
%
% Output parameters
%
%   state_dot: change in state
%
% [x; y; z; xdot; ydot; zdot; phi; theta; psi; phidot; thetadot; psidot; xacc; yacc; zacc];
%   current_state.pos = [x; y; z], 
%   current_state.vel = [x_dot; y_dot; z_dot],
%   current_state.rot = [phi; theta; psi], 
%   current_state.omega = [phidot; thetadot; psidot]
%   current_state.rpm = [w1; w2; w3; w4];
%
%************  DYNAMICS ************************

psi = state(9);
theta = state(8);
phi = state(7);

Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
R = Rz*Ry*Rx;

lin_acc = (1/params.mass)*R*[0; 0; F] - [0; 0; params.gravity];

ang_acc = params.inertia\M;

state_dot(1:3,1) = state(4:6);
state_dot(4:6,1) = lin_acc;
state_dot(7:9,1) = state(10:12);
state_dot(10:12,1) = ang_acc;
state_dot(13:16,1) = rpm_motor_dot;
    
end