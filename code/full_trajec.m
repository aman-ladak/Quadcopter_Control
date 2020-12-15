function [full_trajectory, t1,t2,t3,t4,t5] = full_trajec(dt, des_hover1, des_track, des_hover2, des_land, dir)

%States
Idle = 0;
Takeoff = 1;
Hover = 2;
Tracking = 3;
Hover_2 = 4;
Land = 5;

%Initialize
trajectory = zeros(1,15); %initial null trajectory
t0 = 0;
init_waypoint = [0; 0; 0; 0];

%Generate State Trajectories
[takeoff, f1, tot1] = state_trajectories(trajectory, Takeoff, t0, dt, init_waypoint, des_hover1, dir);
[hover, f2, tot2] = state_trajectories(trajectory, Hover, tot1, dt, f1, des_hover1, dir);
[tracking, f3, tot3] = state_trajectories(trajectory, Tracking, tot2, dt, f2, des_track, dir);
[hover_2, f4, tot4] = state_trajectories(trajectory, Hover_2, tot3, dt, f3, des_hover2, dir);
[land, f5, tot5] = state_trajectories(trajectory, Land, tot4, dt, f4, des_land, dir);

% total_t = tot5;

t1 = tot1;
t2 = tot2-tot1;
t3 = tot3-tot2;
t4 = tot4-tot3;
t5 = tot5-tot4;

full_trajectory = cell(1,5);
full_trajectory{Takeoff} = takeoff;
full_trajectory{Hover} = hover;
full_trajectory{Tracking} = tracking;
full_trajectory{Hover_2} = hover_2;
full_trajectory{Land} = land;

end

