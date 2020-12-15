function [trajectory, final_waypoint, time] = state_trajectories(trajectory, state, time, dt, init_waypoint, des_waypoint, dir)
%trajectory = generated trajectory for state
%state = robot state
%dt = time step
%time = time surpassed
%init_waypoint = Trajectory initial waypoint
%des_waypoint = Trajectory desired final waypoint
%dir = trajectory tracking type (dir=1 -> quadratic; dir=2 -> impulse)

if state == 0 %Idle
    time = 0;
    final_waypoint = [0; 0; 0; 0];
end

if state == 1 %Takeoff
    h = des_waypoint(3,1) - init_waypoint(3,1); %predetermined hover height (z)
    r = 10; %design choice: achieve 1m per 10sec in z direction (10sec/1m)
    t_interval = h*r;
    waypoint_times = 0:dt:t_interval;
    time = t_interval;
    waypoints = zeros(4,length(waypoint_times));

    if mod((length(waypoint_times)-1),2) == 0
        t_crit = (length(waypoint_times)-1)*0.5;
    else
        t_crit = ((length(waypoint_times)-1)*0.5) -0.5;
    end
        
    for i=1:t_crit
        A = [(time^2)/4 time/2; (time^2)/4 -time/4];
        B = [h/2; h/2];
        cof = A\B;
        a = cof(1,1);
        b = cof(2,1);
        t = waypoint_times(i);
        waypoints(3,i) = a*(t^2) + b*t;
        waypoints(3,i) = round(waypoints(3,i),10);
        vz(1,i) = 2*a*t + b;
    end
    for j = (t_crit + 1):(length(waypoint_times))
        A = [(time/2)^2 (time/2) 1; (time)^2 time 1; (3*time/2)^2 (3*time/2) 1];
        B = [h/2; h; h/2];
        cof = A\B;
        a = cof(1,1);
        b = cof(2,1);
        c = cof(3,1);
        t = waypoint_times(j);
        waypoints(3,j) = a*(t^2) + b*t + c;
        waypoints(3,j) = round(waypoints(3,j),10);
        vz(1,j) = 2*a*t + b;
    end
    t_step = (time/dt) + 1;
    t_step = round(t_step);
    trajectory = state_trajectory_planner(42, waypoints, t_step, waypoint_times, dt, vz);
    final_waypoint = waypoints(1:4,length(waypoints));
end

if state == 2 %Hover
    h = des_waypoint(3,1);
    ti = time;
    t_interval = 4; %design choice: hover time of 4sec
    waypoint_times = ti:dt:ti+t_interval; 
    time = time + t_interval;
    
    x = init_waypoint(1,1) * ones(1,length(waypoint_times));
    y = init_waypoint(2,1) * ones(1,length(waypoint_times));
    psi = init_waypoint(4,1) * ones(1,length(waypoint_times));
    z = h*ones(1,length(waypoint_times));
    waypoints = [x; y; z; psi];
    vz = 0;
    t_step = (time-ti)/dt + 1;
    t_step = round(t_step);
    trajectory = state_trajectory_planner(41, waypoints, t_step, waypoint_times, dt, vz);
    final_waypoint = waypoints(1:4,length(waypoints));
end

if state == 3 %Tracking
    if dir == 1
        r = 10; %design choice: achieve 1m per 10sec in tracking direction (10sec/1m)
        ti = time;
        xi = init_waypoint(1,1);
        yi = init_waypoint(2,1);
        zi = init_waypoint(3,1);
        psii = init_waypoint(4,1);
        xf = des_waypoint(1,1);
        yf = des_waypoint(2,1);
        zf = des_waypoint(3,1);
        psif = des_waypoint(4,1);

        dx = abs(xf - xi);
        dy = abs(yf - yi);
        dz = abs(zf - zi);
        t_interval = real((sqrt(dx^2 + dy^2 + dz^2)))*r;
        waypoint_times = ti:dt:ti+t_interval;
        time = time + t_interval;
        if mod((length(waypoint_times)-1),2) == 0
            t_crit = (length(waypoint_times)-1)*0.5;
        else
            t_crit = ((length(waypoint_times)-1)*0.5) -0.5;
        end

        waypoints(1:4,1) = init_waypoint;
        for i=2:t_crit
            A = [((ti-(t_interval/2))^2) (ti-(t_interval/2)) 1; (ti^2) ti 1; ((ti+(t_interval/2))^2) (ti+(t_interval/2)) 1];
            t = waypoint_times(i);

            Bx = [(xf+xi)/2; xi; (xf+xi)/2];
            cofx = A\Bx;
            ax = cofx(1,1);
            bx = cofx(2,1);
            cx = cofx(3,1);
            waypoints(1,i) = ax*(t^2) + bx*t + cx;
            vx(1,i) = 2*ax*t + bx;

            By = [(yf+yi)/2; yi; (yf+yi)/2];
            cofy = A\By;
            ay = cofy(1,1);
            by = cofy(2,1);
            cy = cofy(3,1);
            waypoints(2,i) = ay*(t^2) + by*t + cy;
            vy(1,i) = 2*ay*t + by;

            Bz = [(zf+zi)/2; zi; (zf+zi)/2];
            cofz = A\Bz;
            az = cofz(1,1);
            bz = cofz(2,1);
            cz = cofz(3,1);
            waypoints(3,i) = az*(t^2) + bz*t + cz;
            vz(1,i) = 2*az*t + bz;

            Bpsi = [(psif+psii)/2; psii; (psif+psii)/2];
            cofpsi = A\Bpsi;
            apsi = cofpsi(1,1);
            bpsi = cofpsi(2,1);
            cpsi = cofpsi(3,1);
            waypoints(4,i) = apsi*(t^2) + bpsi*t + cpsi;
            vpsi(1,i) = 2*apsi*t + bpsi; 
        end

        for j = (t_crit + 1):(length(waypoint_times))
            A = [((ti+(t_interval/2))^2) (ti+(t_interval/2)) 1; ((ti+t_interval)^2) (ti+t_interval) 1; ((ti+(3*t_interval/2))^2) (ti+(3*t_interval/2)) 1];
            t = waypoint_times(j);

            Bx = [(xf+xi)/2; xf; (xf+xi)/2];
            cofx = A\Bx;
            ax = cofx(1,1);
            bx = cofx(2,1);
            cx = cofx(3,1);
            waypoints(1,j) = ax*(t^2) + bx*t + cx;
            vx(1,j) = 2*ax*t + bx;

            By = [(yf+yi)/2; yf; (yf+yi)/2];
            cofy = A\By;
            ay = cofy(1,1);
            by = cofy(2,1);
            cy = cofy(3,1);
            waypoints(2,j) = ay*(t^2) + by*t + cy;
            vy(1,j) = 2*ay*t + by;

            Bz = [(zf+zi)/2; zf; (zf+zi)/2];
            cofz = A\Bz;
            az = cofz(1,1);
            bz = cofz(2,1);
            cz = cofz(3,1);
            waypoints(3,j) = az*(t^2) + bz*t + cz;
            vz(1,j) = 2*az*t + bz;

            Bpsi = [(psif+psii)/2; psif; (psif+psii)/2];
            cofpsi = A\Bpsi;
            apsi = cofpsi(1,1);
            bpsi = cofpsi(2,1);
            cpsi = cofpsi(3,1);
            waypoints(4,j) = apsi*(t^2) + bpsi*t + cpsi;
            vpsi(1,j) = 2*apsi*t + bpsi; 
        end    

        t_step = (time-ti)/dt + 1;
        t_step = round(t_step);
        trajectory = state_trajectory_tracking_planner(43, waypoints, t_step, waypoint_times, dt, vx, vy, vz, vpsi);
        final_waypoint = waypoints(1:4,length(waypoints));
    end
    if dir == 2
        r = 10; %design choice: achieve 1m per 10sec in tracking direction (10sec/1m)
        ti = time;
        xi = init_waypoint(1,1);
        yi = init_waypoint(2,1);
        zi = init_waypoint(3,1);
        psii = init_waypoint(4,1);
        xf = des_waypoint(1,1);
        yf = des_waypoint(2,1);
        zf = des_waypoint(3,1);
        psif = des_waypoint(4,1);

        dx = abs(xf - xi);
        dy = abs(yf - yi);
        dz = abs(zf - zi);
        t_interval = (sqrt(dx^2 + dy^2 + dz^2))*r;
        waypoint_times = [ti+t_interval];
        time = time + t_interval;

        waypoints(1:4,1) = des_waypoint;
        vz = 0;
        t_step = (time-ti)/dt + 1;
        t_step = round(t_step);
        trajectory = state_trajectory_planner(5, waypoints, t_step, waypoint_times, dt, vz);
        final_waypoint = waypoints;
    end

end

if state == 4 %Hover2
    h = des_waypoint(3,1);
    ti = time;
    t_interval = 4; %design choice: hover time of 4sec
    waypoint_times = ti:dt:ti+t_interval; 
    time = time + t_interval;
    
    x = init_waypoint(1,1) * ones(1,length(waypoint_times));
    y = init_waypoint(2,1) * ones(1,length(waypoint_times));
    psi = init_waypoint(4,1) * ones(1,length(waypoint_times));
    
    z = h*ones(1,length(waypoint_times));
    waypoints = [x; y; z; psi];
    vz = 0;
    t_step = (time-ti)/dt + 1;
    trajectory = state_trajectory_planner(41, waypoints, t_step, waypoint_times, dt, vz);
    final_waypoint = waypoints(1:4,length(waypoints));
end

if state == 5 %Land
    h = init_waypoint(3,1) - des_waypoint(3,1); %height to land
    r = 10; %design choice: achieve 1m per 10sec in z direction (10sec/1m)
    ti = time;
    t_interval = h*r;
    waypoint_times = ti:dt:ti+t_interval;
    time = time+ t_interval;

    x = init_waypoint(1,1) * ones(1,length(waypoint_times));
    y = init_waypoint(2,1) * ones(1,length(waypoint_times));
    psi = init_waypoint(4,1) * ones(1,length(waypoint_times));
    z = zeros(1, length(waypoint_times));
    waypoints = [x; y; z; psi]; 
    
    if mod((length(waypoint_times)-1),2) == 0
        t_crit = (length(waypoint_times)-1)*0.5;
    else
        t_crit = ((length(waypoint_times)-1)*0.5) -0.5;
    end
    
    for i=1:t_crit
        A = [((ti-(t_interval/2))^2) (ti-(t_interval/2)) 1; (ti^2) ti 1; ((ti+(t_interval/2))^2) (ti+(t_interval/2)) 1];
        B = [h/2; h; h/2];
        cof = A\B;
        a = cof(1,1);
        b = cof(2,1);
        c = cof(3,1);
        t = waypoint_times(i);
        waypoints(3,i) = a*(t^2) + b*t + c;
        vz(1,i) = 2*a*t + b;
    end
    for j = (t_crit + 1):(length(waypoint_times))
        A = [((ti+(t_interval/2))^2) (ti+(t_interval/2)) 1; ((ti+t_interval)^2) (ti+t_interval) 1; ((ti+(3*t_interval/2))^2) (ti+(3*t_interval/2)) 1];
        B = [h/2; 0; h/2];
        cof = A\B;
        a = cof(1,1);
        b = cof(2,1);
        c = cof(3,1);
        t = waypoint_times(j);
        waypoints(3,j) = a*(t^2) + b*t + c;
        vz(1,j) = 2*a*t + b;
    end   
    t_step = (time-ti)/dt + 1;
    t_step = round(t_step);
    trajectory = state_trajectory_planner(42, waypoints, t_step, waypoint_times, dt, vz);
    final_waypoint = waypoints(1:4,length(waypoints));
end
end
    