%Quadratic Trajectory Tracking

% des_hover1 = [0; 0; 1; 0];
% des_track = [1; 0; 1; 0];
% des_hover2 = [1; 0; 1; 0];
% des_land = [1; 0; 0; 0];

% [full_trajectory] = state_machine(des_hover1, des_track, des_hover2, des_land, 4, 1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Impulse Trajectory Tracking

%Test Waypoints Set A
des_hover1 = [0; 0; 1; 0];
des_track = [0; 0; 0.1; 0];
des_hover2 = [0; 0; 0.1; 0];
des_land = [0; 0; 0; 0];

%Test Waypoints Set B
% des_hover1 = [0; 0; 1; 0];
% des_track = [0; 0; 0.1; 0.261799];
% des_hover2 = [0; 0; 0.1; 0.261799];
% des_land = [0; 0; 0; 0.261799];

[full_trajectory] = state_machine(des_hover1, des_track, des_hover2, des_land, 5, 2);
