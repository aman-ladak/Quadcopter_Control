## TEST.m
TEST.m can be used to run a simulation of the robot with a trajectory derived from either a quadratic formulation of continuous waypoints or a single desired waypoint like an impulse.

## Description
The state machine encompasses 5 states: Idle, Takeoff, Hover, Tracking, and Land. Trajectories are defined for each active state in state_trajectories.m, which first defines waypoints for the specific states, and makes use of state_trajectory_planner.m (for Takeoff, Hover, single waypoint Tracking, and Land) and state_trajectory_tracking_planner.m (for continuous waypoint Tracking) to generate trajectories based on the waypoints and their associated times. 

Times are defined based on design decisions as follows:

- Takeoff
  - Achieve 1 meter change in z-direction per 10 seconds
- Hover
  - 4 seconds
- Tracking
  - Achieve 1 meter change in tracking direction per 10 seconds    
- Land
  - Achieve 1 meter change in z-direction per 10 seconds

The call to the state_machine.m is generated with the desired hover, tracking and landing destinations in terms of waypoints which first calls on full_trajec.m, which then calls on state_trajectories.m to generate the trajectories based on the desired waypoints. full_trajec.m assembles the trajectories together in a Matlab cell data structure, along with times associated with each state trajectory. The State Machine simulation then begins by transitioning from the Idle to Takeoff state until the desired hover height is achieved. If attained based on the error, the robot transitions to the hover state for 4 seconds. Once this is complete, the robot will then track the desired waypoint. Once achieved, the robot transitions into the hover state for 4 seconds. Upon completion, the robot transitions to the land state to go to the ground. 

## Quadcopter System Diagram
![system-diagram](https://user-images.githubusercontent.com/76025995/102175632-e1779500-3e6d-11eb-8078-a3860ad6424b.jpg)

## State Machine Diagram
![state_machine_diagram](https://user-images.githubusercontent.com/76025995/102175886-459a5900-3e6e-11eb-9845-9aa12f04f781.jpg)
