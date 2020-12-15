function [F, acc] = position_controller(current_state, desired_state, params, question)

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
%   F: u1 or thrust
%
%   acc: will be stored as desired_state.acc = [xdotdot; ydotdot; zdotdot]
%
%************  POSITION CONTROLLER ************************

% PD gains
if question == 2
%     Kp1 = 17; 
%     Kp2 = 17; 
%     Kp3 = 20; 
%     Kd1 = 7.2;
%     Kd2 = 6.6;
%     Kd3 = 9;
    
    Kp1 = 20; 
    Kp2 = 17; 
    Kp3 = 20; 
    Kd1 = 9.0;
    Kd2 = 6.6;
    Kd3 = 12;
end

if question == 3
%     Kp1 = 17; 
%     Kp2 = 17; 
%     Kp3 = 20; 
%     Kd1 = 7.2;
%     Kd2 = 6.6;
%     Kd3 = 9;
    
    Kp1 = 17; 
    Kp2 = 17; 
    Kp3 = 40; 
    Kd1 = 7.2;
    Kd2 = 6.6;
    Kd3 = 20;
end

if question == 4
    Kp1 = 17; 
    Kp2 = 17; 
    Kp3 = 20; 
    Kd1 = 7.2;
    Kd2 = 6.6;
    Kd3 = 9;
end

if question == 5
    Kp1 = 20;
    Kp2 = 20; 
    Kp3 = 18; 
    Kd1 = 8;
    Kd2 = 8;
    Kd3 = 9;
%     
%     Kp1 = 20;
%     Kp2 = 20; 
%     Kp3 = 10; 
%     Kd1 = 8;
%     Kd2 = 8;
%     Kd3 = 19;
    
end

Kp = [Kp1 0 0; 0 Kp2 0; 0 0 Kp3];
Kd = [Kd1 0 0; 0 Kp2 0; 0 0 Kd3];

b_3_tr = [0 0 1];
% u_ff = desired_state(13:15); %acceleration

e_xyz = current_state.pos - desired_state.pos; %position error
edot_xyz = current_state.vel - desired_state.vel; %velocity error

edotdot_xyz = -Kp*e_xyz - Kd*edot_xyz;

desired_state.acc = desired_state.acc + edotdot_xyz;

F = params.mass*b_3_tr*([0; 0; params.gravity] + desired_state.acc);
acc = desired_state.acc;
% acc = err_dotdot_xyz + desired_state.acc

end
