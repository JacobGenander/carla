clear
clc

state1 = [0,0,0,0,0];
action = [0.1, 0.2];

for c = 1:100
    state_next = kinematic_model(state1,action);
    state1 = state_next;
    disp(state_next);
end
    