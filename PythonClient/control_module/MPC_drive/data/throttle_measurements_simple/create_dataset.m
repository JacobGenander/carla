load experiments

exps = {experiment1; experiment2; experiment3; experiment4; experiment5;...
    experiment6; experiment7; experiment8; experiment9; experiment10};

% Use this plot to select the range in which all experiments have useful
% data (I decided to discard experiment 1)
% for i=2:10
%     exp = exps{i};
%     a = exp.acceleration;
%     throttle = exp.throttle;
%     v = exp.velocity;
%     subplot(3,3,i-1);
%     
%     plot(throttle);
%     hold on;
%     plot(a);
%     plot(v);
%     grid minor;
%     legend throttle acceleration speed
%     title(['experiment' num2str(i)])
% end

% These are the values I obtained via visual inspection of the above plot
start = 205;
safe_end = 317;

% This plot checks the range specified above
% for i=2:10
%     exp = exps{i};
%     a = exp.acceleration;
%     throttle = exp.throttle;
%     v = exp.velocity;
%     subplot(3,3,i-1);
%     
%     plot(throttle(start:safe_end));
%     hold on;
%     plot(a(start:safe_end));
%     plot(v(start:safe_end));
%     grid minor;
%     legend throttle acceleration speed
%     title(['experiment' num2str(i)])
% end

a = [];
v = [];
throttle = [];
for i = 2:10
    exp = exps{i};
    a = [a; exp.acceleration(start:safe_end)];
    v = [v; exp.velocity(start:safe_end)];
    throttle = [throttle; exp.throttle(start:safe_end)];
end

inputs = [a v v.^2]';
targets = throttle';

% This plots the dataset that will be created
% plot(inputs(:,1)) % acceleration
% hold on;
% plot(inputs(:,2)) % velocityy
% yyaxis right
% plot(targets) % throttle
% legend a v throttle

save neural_net_dataset inputs targets  
clear;
