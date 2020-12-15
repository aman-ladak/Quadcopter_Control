function [M] = attitude_controller(current_state,desired_state,params,question)

% Input parameters
% 
%   current_state: The current state of the robot with the following fields:
%   current_state.pos = [x; y; z], 
%   current_state.vel = [x_dot; y_dot; z_dot],
%   current_state.rot = [phi; theta; psi], 
%   current_state.omega = [phidot; thetadot; psidot]
%   current_state.rpm = [w1; w2; w3; w4];
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
%   question: Question number
%
% Output parameters
%
%   M: u2 or moment [M1; M2; M3]
%
%************  ATTITUDE CONTROLLER ************************

%PD gains
if question==2
%     Kpphi = 210;
%     Kptheta = 200;
%     Kppsi = 80;
%     Kdphi = 32;
%     Kdtheta = 32;
%     Kdpsi = 17.88;
        
    Kpphi = 210;
    Kptheta = 210;
    Kppsi = 80;
    Kdphi = 32;
    Kdtheta = 35;
    Kdpsi = 17.88;
end

if question==3
    Kpphi = 190;
    Kptheta = 198;
    Kppsi = 80;
    Kdphi = 30;
    Kdtheta = 30;
    Kdpsi = 17.88;
    
end

if question==4
    Kpphi = 210;
    Kptheta = 198;
    Kppsi = 80;
    Kdphi = 32;
    Kdtheta = 32;
    Kdpsi = 17.88;
end

if question==5
    Kpphi = 190;
    Kptheta = 190;
    Kppsi = 70;
    Kdphi = 30;
    Kdtheta = 30;
    Kdpsi = 18;
    
%     Kpphi = 190;
%     Kptheta = 190;
%     Kppsi = 20;
%     Kdphi = 30;
%     Kdtheta = 30;
%     Kdpsi = 18;
end

e_rot = [current_state.rot(1)-desired_state.rot(1); current_state.rot(2)-desired_state.rot(2); current_state.rot(3)-desired_state.rot(3)];

e_rot_dot = [current_state.omega(1)-desired_state.omega(1); current_state.omega(2)-desired_state.omega(2); current_state.omega(3)-desired_state.omega(3)];

Kp = [Kpphi 0 0; 0 Kptheta 0; 0 0 Kppsi];
Kd = [Kdphi 0 0; 0 Kdtheta 0; 0 0 Kdpsi];

M = params.inertia * (-Kp*e_rot - Kd*e_rot_dot + desired_state.acc);
% + feedfwd;

end

