clear
clc

%% Preparations
% Load data from the experiment (as an example of trajectory for the MPC to
% follow)
full_experiment = load('../data/data_2018_04_11/measurements'); 

% Choose part of the trajectory 
start = 40;
end_ = start+20;
traj = [full_experiment.x(start:end_),full_experiment.y(start:end_)];
psi0 = full_experiment.psi(start);
v0 = full_experiment.v(start);


%% Compute optimal input
% MPC prediction step
[u_optimal, z_optimal, fval, output] = CARLA_MPC([traj(1,:), psi0, v0], traj);   


%% Plot the results
% Convert predicted position in meters to pixels
x_map = (z_optimal(:,3)+16.43)/0.1643;
y_map = (z_optimal(:,4)+16.43)/0.1643;

% Convert position setpoints from CARLA from meters to pixels
traj_map = (traj+16.43)/0.1643;

% Convert all positon setpoints from CARLA from meters to pixels
entire_trajectory = [full_experiment.x, full_experiment.y];
entire_trajectory_map = (entire_trajectory+16.43)/0.1643;

% Plot entire trajectory from CARLA
subplot(1,3,1);
plot(entire_trajectory_map(:,1), entire_trajectory_map(:,2), 'ko', ...
     'MarkerFaceColor', 'k');
hold on;

% Plot setpoints
plot(traj_map(:,1), traj_map(:,2),'bo','MarkerFaceColor', 'b', 'MarkerSize', 8)

% Plot predicted trajectory
plot(x_map,y_map,'ro-', 'MarkerFaceColor', 'r')

% Set title, y axis direction, aspect ratio, grid, labels, legends
title("Loss: " + num2str(fval) + ...
      "   Feasibility: " + num2str(sum(output.constrviolation)))
set(gca,'Ydir','reverse')
grid minor;
xlabel x;
ylabel y;
legend 'entire experiment' 'reference points' 'predicted positions';

% Plot steering wheel angle
subplot(1,3,2);
plot(u_optimal(:,1),'ko-','MarkerFaceColor', 'k');
title('optimal steering angle');
grid minor;
axis([1 21 -inf inf]);

% Plot throttle and brake
subplot(1,3,3);
plot(u_optimal(:,2),'bo-','MarkerFaceColor', 'b'); hold on;
plot(u_optimal(:,3),'ro-','MarkerFaceColor', 'r'); 
title('optimal inputs');
legend throttle brake
grid minor;
axis([1 21 -inf inf]);




