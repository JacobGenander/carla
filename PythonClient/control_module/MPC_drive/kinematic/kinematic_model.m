function [new_state] = kinematic_model(state, action)
%state = [x, y, psi, speed, beta]
%action = [detla_f,a]
%lf and lr represent the distance from the center of the mass of the 
%vehicle to the front and rear axles. The number is for Hyundai Azera.
%x and y are the coordinates of the center of mass in an inertial frame (X, Y ).
%ψ is the inertial heading and v is the speed of the vehicle.
%β is the angle of the current velocity of the center of mass 
%with respect to the longitudinal axis of the car. 
%a is the acceleration of the center of mass in the same direction as the velocity. 
%The control inputs are the front and rear steering angles δf and a.

% Model parameters
lr = 1.738;
lf = 1.105;
delta_t = 1;

% Break down state and action
x = state(1);
y = state(2);
psi = state(3);
speed = state(4);

delta_f = action(1);
a = action(2);

% Compute beta
beta = atan(lr/(lr+lf)*tan(delta_f));

% Compute next state
x_next = x + delta_t * speed * cos(psi+beta);
y_next = y + delta_t * speed * sin(psi+beta);
psi_next = psi + delta_t * speed * sin(beta)/lr;
speed_next = speed + delta_t * a;
new_state = [x_next, y_next, psi_next, speed_next];

end