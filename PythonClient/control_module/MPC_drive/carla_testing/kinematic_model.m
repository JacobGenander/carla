function [new_state] = kinematic_model(state, action)
% This function computes the next state of the car, given the current state
% and action taken.
%
% The computation uses a discretized kinematic bicycle model, where 
% positions, velocities, and accelerations are computed for the pivot point
% of the car (center of the rear axle).
%
%
% STATE VECTOR
% -------------------------------------------------------------------------
% state = [x, y, psi, speed]
%
% x, y: coordinates of the pivot point of the car (center of the rear
%   axle) in the world frame [m].
% ψ: angle between the x-axis of the car (pointing forward, in the
%   direction of the chassis of the vehicle) and the x-axis of the world 
%   frame [rad].
% speed: magnitude of the velocity vector of the pivot point of the
% vehicle [m/s].
%
%
% ACTION VECTOR
% -------------------------------------------------------------------------
% action = [delta_f,a]
%
% a: magnitude of the acceleration vector of the pivor point of the
%   vehicle [m/s²].
% \delta_f: front wheel steering angle [rad].
%
%
% PARAMETERS
% -------------------------------------------------------------------------
% lf: distance from the pivot point of the car to the front axle [m]. The 
%   number 2.339*2 is our estimate for the car in CARLA.
% delta_t: step-size for the discretization of the continuous kinematic 
%   model [s].


    % Model parameters
    lf = 2.339*2;
    delta_t = 0.1;

    % Break down state and action
    x = state(1);
    y = state(2);
    psi = state(3);
    speed = state(4);
    delta_f = action(1);
    a = action(2);

    % Compute next state
    x_next = x + delta_t * speed * cos(psi);
    y_next = y + delta_t * speed * sin(psi);
    psi_next = psi + delta_t * speed * tan(delta_f)/lf;
    speed_next = speed + delta_t * a;
    new_state = [x_next, y_next, psi_next, speed_next];

end