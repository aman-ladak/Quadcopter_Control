function trajectory_state = state_trajectory_tracking_planner(question, waypoints, max_iter, waypoint_times, time_step,vx,vy,vz,vpsi)
% Output parameters
%
%   trajectory_sate: [15 x max_iter] output trajectory as a matrix of states:
%   [x; y; z; xdot; ydot; zdot; phi; theta; psi; phidot; thetadot; psidot; xacc; yacc; zacc];

trajectory_state = zeros(15,max_iter);
% 15 rows for: [x; y; z; xdot; ydot; zdot; phi; theta; psi; phidot; thetadot; psidot; xacc; yacc; zacc];
current_waypoint_number = 1;

if question == 43
    for iter = 1:max_iter
    if (current_waypoint_number<length(waypoint_times))
        if((iter*time_step)>(waypoint_times(current_waypoint_number+1) - waypoint_times(1)))
            current_waypoint_number = current_waypoint_number + 1;
        end
    end
    trajectory_state(1:3,iter) = waypoints(1:3,current_waypoint_number); %x,y,z
    trajectory_state(9,iter) = waypoints(4,current_waypoint_number); %yaw
    trajectory_state(4,iter) = vx(1,current_waypoint_number);
    trajectory_state(5,iter) = vy(1,current_waypoint_number);
    trajectory_state(6,iter) = vz(1,current_waypoint_number);
    trajectory_state(12,iter) = vpsi(1,current_waypoint_number);
    end

end

end
