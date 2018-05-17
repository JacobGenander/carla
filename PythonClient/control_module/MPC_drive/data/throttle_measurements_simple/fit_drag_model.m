%% Load data
% Load experiment
exp = experiment2;
t = exp.time/1000;
a = exp.acceleration;
throttle = exp.throttle;
v = exp.velocity;

% Crop to interesting part
start = 200;
end_ = 500;
t = t(start:end_);
a = a(start:end_);
throttle = throttle(start:end_);
v = v(start:end_);

% Plot
plot(t,throttle);
hold on;
plot(t,a);
plot(t,v);
grid minor;
legend throttle acceleration speed

%% Fit data visually

clf;
plot(t,a); hold on;

i = 1;
for k = linspace(0.0211,0.022,20)
    k
    model_a{i} = a + k*v.^2;
    plot(t,model_a{i});
    i = i + 1;
end
grid minor;

