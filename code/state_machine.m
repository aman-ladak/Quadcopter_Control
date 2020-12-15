function [full_trajectory] = state_machine(des_hover1, des_track, des_hover2, des_land, q, dir)
%Return Full Trajectory Generated

warning off;
%Params
params = struct(...
    'mass',                   0.770, ...
    'gravity',                9.80665, ...
    'arm_length',           0.1103, ... %d
    'motor_spread_angle',     0.925, ...
    'thrust_coefficient',     8.07e-9, ... %cT
    'moment_scale',           1.3719e-10, ... %cQ
    'motor_constant',        36.5, ... %km
    'rpm_min',             3000, ...
    'rpm_max',            20000, ...
    'inertia',            diag([0.0033 0.0033 0.005]),...
    'COM_vertical_offset',                0.05);

%States
Idle = 0;
Takeoff = 1;
Hover = 2;
Tracking = 3;
Hover_2 = 4;
Land = 5;

time_step = 0.005; %sec

%Trajectory
if q == 4
    [full_trajectory,t1,t2,t3,t4,t5] = full_trajec(time_step, des_hover1, des_track, des_hover2, des_land, dir);
    question = 4;
elseif q == 5
    [full_trajectory,t1,t2,t3,t4,t5] = full_trajec(time_step, des_hover1, des_track, des_hover2, des_land, dir);
    question = 5;
end

%Sim Params
time_initial = 0;
total_t = t1+t2+t3+t4+t5; %38 for original
% time_final = t+7;
time_final = total_t;
time_vec = time_initial:time_step:time_final;
max_iter = length(time_vec);

iter1 = length(time_initial:time_step:t1);
iter2 = length(t1:time_step:t1+t2);
iter3 = length(t1+t2:time_step:t1+t2+t3);
iter4 = length(t1+t2+t3:time_step:t1+t2+t3+t4);
iter5 = length(t1+t2+t3+t4:time_step:total_t);

check1 = 0;
check2 = 0;
check3 = 0;
check4 = 0;
% check5 = 0;

epsilon = 0.1; %pose error tolerance

state = zeros(16,1);
state(1) = 0;        %x
state(2) = 0;        %y
state(3) = 0;        %z
state(4) = 0;        %xdot
state(5) = 0;        %ydot
state(6) = 0;        %zdot
state(7) = 0;        %phi
state(8) = 0;        %theta
state(9) = 0;        %psi
state(10) = 0;        %phidot 
state(11) = 0;        %thetadot
state(12) = 0;        %psidot
state(13:16) = 0;     %rpm


actual_state_matrix = zeros(15, max_iter);
actual_state_matrix(:,1) = vertcat(state(1:12), 0, 0, 0);

actual_desired_state_matrix = zeros(15, max_iter);

%Load takeoff trajectory
disp('Loading Takeoff Trajectory');
takeoff_traj = full_trajectory(1);
takeoff_traj = cell2mat(takeoff_traj);

current_state.pos = state(1:3);
current_traj = takeoff_traj;

for iter = 1:max_iter-1
    %Convert current state to stuct for control functions
    current_state.pos = state(1:3);
    current_state.vel = state(4:6);
    current_state.rot = state(7:9);
    current_state.omega = state(10:12);
    current_state.rpm = state(13:16);

    % Get desired state from matrix, put into struct for control functions
    desired_state.pos = current_traj(1:3,iter);
    desired_state.vel = current_traj(4:6,iter);
    desired_state.rot = current_traj(7:9,iter);
    desired_state.omega = current_traj(10:12,iter);
    desired_state.acc = current_traj(13:15,iter);

    [F, desired_state.acc] = position_controller(current_state, desired_state, params, question);

    [desired_state.rot, desired_state.omega] = attitude_planner(desired_state, params);

    M = attitude_controller(current_state, desired_state, params, question);

    [F_actual, M_actual, rpm_motor_dot] = motor_model(F, M, current_state.rpm, params);

    timeint = time_vec(iter:iter+1);
    [tsave, xsave] = ode45(@(t,s) dynamics(params, s, F_actual, M_actual, rpm_motor_dot), timeint, state);
    state    = xsave(end, :)';
    acc  = (xsave(end,4:6)' - xsave(end-1,4:6)')/(tsave(end) - tsave(end-1));

    % Update desired state matrix
    actual_desired_state_matrix(1:3,iter+1) =  desired_state.pos;
    actual_desired_state_matrix(4:6, iter+1) = desired_state.vel;
    actual_desired_state_matrix(7:9, iter+1) = desired_state.rot;
    actual_desired_state_matrix(10:12, iter+1) = desired_state.omega;
    actual_desired_state_matrix(13:15, iter+1) = desired_state.acc;

    % Update actual state matrix
    actual_state_matrix(1:12, iter+1) = state(1:12);
    actual_state_matrix(13:15, iter+1) = acc;
    
    if ((iter == iter1-1) && (check1==0))
        if (abs(takeoff_traj(3,iter1) - actual_state_matrix(3,iter1)) <= epsilon)
            disp('Achieved Takeoff State Tolerance. Transitioning to Hover State');
            disp('Loading Hover Trajectory');
            hover_traj = full_trajectory(2);
            hover_traj = cell2mat(hover_traj);
            current_traj = horzcat(current_traj, hover_traj);
            check1 = 1;
        else
            disp('Not able to achieve error tolerance')
            break
        end
    end
    
    if ((iter == iter1+iter2-1) && (check2==0) && (check1==1))
        disp('Achieved Hover State. Transitioning to Tracking State');
        disp('Loading Tracking Trajectory');
        tracking_traj = full_trajectory(3);
        tracking_traj = cell2mat(tracking_traj);
        current_traj = horzcat(current_traj, tracking_traj);
        check2 = 1;
    end
    
    if ((iter == iter1+iter2+iter3-1) && (check3==0) && (check2==1))
        disp('Achieved Tracking State. Transitioning to Hover State');
        disp('Loading Hover2 Trajectory');
        hover2_traj = full_trajectory(4);
        hover2_traj = cell2mat(hover2_traj);
        current_traj = horzcat(current_traj, hover2_traj);
        check3 = 1;
    end
    
    if ((iter == iter1+iter2+iter3+iter4-1) && (check4==0) && (check3==1))
        disp('Achieved Hover2 State. Transitioning to Land State');
        disp('Loading Land Trajectory');
        land_traj = full_trajectory(5);
        land_traj = cell2mat(land_traj);
        current_traj = horzcat(current_traj, land_traj);
        check4 = 1;
    end  
    
end
    
disp('Achieved Full State Program.');
plot_quadrotor_errors(actual_state_matrix, actual_desired_state_matrix, time_vec)

end

