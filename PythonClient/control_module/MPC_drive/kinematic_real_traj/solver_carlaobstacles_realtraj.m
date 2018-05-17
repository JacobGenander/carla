clear
clc

USE_LOSS_GRADIENT = false;

% Load data from the experiment
full_experiment = load('../data/data_2018_04_11/measurements'); 

% Create figure for later plotting
f = figure;
set(f, 'units', 'normalized','pos', [0.1526 0.4574 0.7823 0.3861])

% Load the distance map to all objects (in order to simplify the usage of
% the MPC function, this is also loaded inside it, just once)
load ../data/distance_maps/dMaps

%% MPC prediction step
for start = 1:10:length(full_experiment.x)-20
    
    % Crop trajectory
    end_ = start+20;
    traj = [full_experiment.x(start:end_),full_experiment.y(start:end_)];
    psi = full_experiment.psi(start:end_);
    v = full_experiment.v(start:end_);
    steer = full_experiment.steer(start:end_);
    delta_f = full_experiment.delta_f(start:end_);
    throttle = full_experiment.throttle(start:end_);
    a_v = full_experiment.a_v(start:end_);
    a_x = full_experiment.a_x(start:end_);
    a_y = full_experiment.a_y(start:end_);
    a = sqrt(a_x.^2+a_y.^2);
    
    [u_optimal, z_optimal, fval, output] = MPC([traj(1,:), psi(1), v(1)], traj);
    

%% Plot the results
    clf;
    
    % Convert predicted position in meters to pixels
    x_map = (z_optimal(:,3)+16.43)/0.1643;
    y_map = (z_optimal(:,4)+16.43)/0.1643;
    
    % Convert current slice of position setpoints from CARLA from meters to 
    % pixels
    traj_map = (traj+16.43)/0.1643;
    
    % Convert all positon setpoints from CARLA from meters to pixels
    entire_trajectory = [full_experiment.x, full_experiment.y];
    entire_trajectory_map = (entire_trajectory+16.43)/0.1643;
    
    % Plot all the obstacles
    subplot(2,4,[1,2,5,6])
    hold on;
    [x_temp,y_temp]=find(dMap2==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    [x_temp,y_temp]=find(dMap3==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    [x_temp,y_temp]=find(dMap4==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    [x_temp,y_temp]=find(dMap5==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    [x_temp,y_temp]=find(dMap6==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    [x_temp,y_temp]=find(dMap7==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    [x_temp,y_temp]=find(dMap8==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    [x_temp,y_temp]=find(dMap9==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    
    % Restrict window to relevant part of the map
    MAP_WINDOW = [600 1000 1100 1500];
    axis(MAP_WINDOW)
    
    % Plot entire trajectory from CARLA
    plot(entire_trajectory_map(:,1), entire_trajectory_map(:,2), 'ko', ...
         'MarkerFaceColor', 'k');
    
    % Plot current slice of setpoints
    plot(traj_map(:,1), traj_map(:,2),'bo','MarkerFaceColor', 'b', 'MarkerSize', 8)
    
    % Plot predicted trajectory
    plot(x_map,y_map,'ro-', 'MarkerFaceColor', 'r')
    
    % Set title, y axis direction, aspect ratio, grid, labels
    title("Loss: " + num2str(fval) + ...
          "   Feasibility: " + num2str(sum(output.constrviolation)))
    set(gca,'Ydir','reverse')
    axis square;
    grid minor;
    xlabel x;
    ylabel y;

    % Plot heading angle
    subplot(2,4,3);
    plot(psi,'bo-','MarkerFaceColor', 'b'); hold on
    plot(z_optimal(:,5),'ro-','MarkerFaceColor', 'r'); 
    title \psi
    grid minor;
    axis([1 20 -pi-0.2 pi+0.2]);
    legend CARLA MPC;

    % Plot speed
    subplot(2,4,4);
    plot(v, 'bo-', 'MarkerFaceColor', 'b'); hold on
    plot(z_optimal(:,6), 'ro-', 'MarkerFaceColor', 'r'); 
    title speed;
    grid minor;
    axis([1 20 -inf inf]);
    legend CARLA MPC;

    % Plot input delta_f
    subplot(2,4,7)
    plot(delta_f(1:end-1), 'bo-', 'MarkerFaceColor', 'b'); hold on
    plot(z_optimal(1:end-1,1), 'ro-', 'MarkerFaceColor', 'r');
    title delta_f
    grid minor 
    axis([1 20 -0.5 1]); % hand-chosen
    legend CARLA MPC

    % Plot input acceleration
    subplot(2,4,8)
    plot(a_v(2:end-1), 'bo-', 'MarkerFaceColor', 'b'); hold on
    plot(z_optimal(1:end-1,2), 'ro-', 'MarkerFaceColor', 'r');
    title acceleration
    grid minor
    axis([1 20 -inf inf]);
    legend CARLA MPC
    
    drawnow;
    waitforbuttonpress;
    
    
end

    