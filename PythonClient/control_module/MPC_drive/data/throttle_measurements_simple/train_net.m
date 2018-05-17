clear; clc;
rng(1);

% Load dataset and create network
load('neural_net_dataset.mat');
net = feedforwardnet(4);

% Train
[net,tr]= train(net,inputs,targets);

% Plot predictions against ground truth
preds = net(inputs);
plot(preds); hold on; plot(targets);