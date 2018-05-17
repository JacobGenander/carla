clear; clc

%% Initialize variables.
filename = '/home/juliano/Chalmers/MPC_drive/data/data_2018_04_11/player_measurements.csv';
delimiter = ',';
startRow = 2;

%% Format for each line of text:
%   column3: double (%f)
%	column4: double (%f)
%   column6: double (%f)
%	column7: double (%f)
%   column8: double (%f)
%	column9: double (%f)
%   column12: double (%f)
%	column18: double (%f)
%   column19: double (%f)
%	column20: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%*s%*s%f%f%*s%f%f%f%f%*s%*s%f%*s%*s%*s%*s%*s%f%f%f%*s%*s%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string', 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');

%% Close the text file.
fclose(fileID);

%% Create output variable
measurements = table(dataArray{1:end-1}, 'VariableNames', {'location_x','location_y','acceleration_x','acceleration_y','acceleration_z','forward_speed','yaw','steer','throttle','brake'});

%% Separate the data using meaningful names
% and change degrees to radians in the heading angle
x = measurements.location_x;
y = measurements.location_y;
a_x = measurements.acceleration_x;
a_y = measurements.acceleration_y;
a_z = measurements.acceleration_z;
v = measurements.forward_speed;
psi = deg2rad(measurements.yaw);
steer = measurements.steer;
throttle = measurements.throttle;
brake = measurements.brake;

%% Process the data
% Front wheel angles
delta_f = deg2rad(steer*70);

% Compute acceleration in the direction of the velocity vector
a_xy = vecnorm([a_x, a_y],2,2);
phase_a_xy = atan2(a_y, a_x);
a_v = a_xy.*cos(phase_a_xy -psi);

% Compare a_v with numerical differentiation of v
plot(a_v(2:end));
hold on
plot(diff(v)/0.1)

%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID...
    dataArray ans lf lr;

%% Save results into .mat file
save measurements.mat






