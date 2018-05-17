% This script generates the neural_net_data.mat file, used by the
% train_net.m script. In order to run this you should have all the .csv
% files for the different episodes in the same folder as this script.
%
% The neural_net_data.mat file is comprised of the input to be fed to the
% network and the targets. 
%
% The input is the acceleration of the pivot point
% in the direction of the car's chassis, the speed of the pivot point, its 
% square, and the steering wheel angle. These are the concatenation of the 
% values for each experiment.
%
% The targets are the throttle and brake values, also concatenated for all
% the experiments.

x = [];
y = [];
a_x = [];
a_y = [];
v = [];
psi = [];
steer = [];
throttle = [];
brake = [];

%% Aggregate all experiments
temp = dir('*.csv');
filenames = {temp.name};
for i = 1:length(filenames)
    safety_margin = 30; % to discard most collisions
    
    [x_, y_, ~, a_x_, a_y_, ~, v_, psi_, steer_, throttle_, brake_] = ...
        load_data(filenames{i});
    x = [x x_(1:end-safety_margin)'];
    y = [y y_(1:end-safety_margin)'];    
    a_x = [a_x a_x_(1:end-safety_margin)'];
    a_y = [a_y a_y_(1:end-safety_margin)'];
    v = [v v_(1:end-safety_margin)'];
    psi = [psi psi_(1:end-safety_margin)'];
    steer = [steer steer_(1:end-safety_margin)'];
    throttle = [throttle throttle_(1:end-safety_margin)'];
    brake = [brake brake_(1:end-safety_margin)'];
    disp([num2str(i) ' done.']);
    
end

%% Pre-process the data
psi = deg2rad(psi);

% Front wheel angles
delta_f = deg2rad(steer*70);

% Compute acceleration in the direction of the velocity vector
a_xy = vecnorm([a_x', a_y'],2,2);
phase_a_xy = atan2(a_y', a_x');
a_v = a_xy.*cos(phase_a_xy -psi');

%% Create file for training
inputs = [a_v'; v; v.^2; delta_f];
targets = [throttle; brake];
save neural_net_data inputs targets