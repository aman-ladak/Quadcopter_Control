function [waypoints, waypoint_times] = lookup_waypoints(question)
%
% Input parameters
%
%   question: which question of the project we are on 
%      Possible arguments for question: 2, 3, 5, 6.2, 6.3, 6.5, 7, 9, 10
%
% Output parameters
%
%   waypoints: of the form [x; y; z; yaw]
% 
%   waypoint_times: [1 x n] vector of times where n is the number of waypoints, 
%   represents the seconds you should be at each respective waypoint
%
%************ LOOKUP WAYPOINTS ************************

if question == 2
    waypoints = [0 0.1 0.2 0.3; 0 0 0 0; 0.5 0.5 0.5 0.5; 0 0 0 0];
    waypoint_times = [0 2 4 6];
end

if question == 3
    waypoints = zeros(4,4);
    waypoint_times = 0:0.01:6;
    for i=1:150
        t = waypoint_times(i);
        waypoints(3,i) = (2/9)*(t^2); 
    end
    for j = 151:450
        t = waypoint_times(j);
        waypoints(3,j) = (-2/9)*(t^2) + (4/3)*t - 1;
    end
    for k = 451:601
        t = waypoint_times(k);
        waypoints(3,k) = (2/9)*(t^2) - (8/3)*t + 8;
    end
end

% 
% if question == 5
%     waypoints = [];
%     waypoint_times = [];
% end


end
