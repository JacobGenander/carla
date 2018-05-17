% This script plots the trajectory, inputs, and states for all the
% experiments present in this directory as .csv files.
%
% Cycle through the different experiments by pressing any key.
%

% Load distance maps (to plot the obstacles)
load ../distance_maps/dMaps

% Get filenames for all .csv files in this directory
temp = dir('*.csv');
filenames = {temp.name};

% Loop through each file, load, and plot
for i = 1:length(filenames)
    [x, y, ~, a_x, a_y, ~, v, psi, steer, throttle, brake] = ...
        load_data(filenames{i});

    psi = deg2rad(psi);

    % Front wheel angles
    delta_f = deg2rad(steer*70);

    % Compute acceleration in the direction of the velocity vector
    a_xy = vecnorm([a_x, a_y],2,2);
    phase_a_xy = atan2(a_y, a_x);
    a_v = a_xy.*cos(phase_a_xy -psi);
    
    
    % Plot trajectory
    clf;
    title(['Episode ' num2str(i)]);
    subplot(2,3,[3 6]);
    x_map = (x+16.43)/0.1643;
    y_map = (y+16.43)/0.1643;

    [x_temp,y_temp]=find(dMap2==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    hold on;
    [x_temp,y_temp]=find(dMap3==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    [x_temp,y_temp]=find(dMap4==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    [x_temp,y_temp]=find(dMap5==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    [x_temp,y_temp]=find(dMap6==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    [x_temp,y_temp]=find(dMap7==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    [x_temp,y_temp]=find(dMap8==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    [x_temp,y_temp]=find(dMap9==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    scatter(x_map, y_map, 10, linspace(0,1,length(x)))
    axis square
    
    % Plot inputs
    subplot(2,3,[1 2]);
    plot(throttle);
    hold on
    plot(brake);
    legend throttle brake;
    axis([0 inf -inf inf]);
    
    % Plot states
    subplot(2,3,[4 5]);
    plot(a_v);
    axis([0 inf -5 5]);
    yyaxis right
    plot(delta_f);
    legend a_v delta_f;
 

    drawnow; 
    waitforbuttonpress;
end