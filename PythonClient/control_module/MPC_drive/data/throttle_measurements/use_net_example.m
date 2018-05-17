% This script shows an example on how to use the network trained by the
% script train_net.m to predict throttle and brake values for new inputs.
clear; clc;

%% Load measurements from .csv file
% Important: these are being loaded as column vectors
[x, y, ~, acceleration_x, acceleration_y, ~, forward_speed, ...
    heading_angle, steer, throttle, brake] = ...
    load_data('episode_1.csv');


%% Pre-process the data

% Convert to radians
psi = deg2rad(heading_angle);
    
% Compute acceleration in the direction of the velocity vector
a_xy = vecnorm([acceleration_x, acceleration_y],2,2); %magnitude
phase_a_xy = atan2(acceleration_y, acceleration_x); %direction
a_v = a_xy.*cos(phase_a_xy -psi); % projection


%% Compute predictions
% Load the network .mat file (should be in the same folder as this script)
load net;

% Populate structure with CARLA measurements (fill these with the real values
% to be used, instead of the ones we read from the .csv file)
state = struct('heading_angle', heading_angle, ...
                'steer', steer, ...
                'a_v', a_v, ...
                'forward_speed', forward_speed);
            
% Evaluate network
[predicted_throttle, predicted_brake] = evaluate_net(net, state);


%% Compare predictions and ground-truth
subplot(2,1,1);
plot(throttle); hold on;
plot(predicted_throttle);

subplot(2,1,2);
plot(brake); hold on;
plot(predicted_brake);