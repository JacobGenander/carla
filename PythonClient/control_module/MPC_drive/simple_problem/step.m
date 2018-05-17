function [new_s] = step(s, u)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    delta_t = 1;

    % Break down state
    x = s(1);
    y = s(2);
    theta = s(3);
    v = s(4);
    
    % Break down input
    u_theta = u(1);
    u_v = u(2);

    new_x = x + delta_t*v*cos(theta);
    new_y = y + delta_t*v*sin(theta);
    new_theta = theta + delta_t*u_theta;
    new_v = v + delta_t*u_v; 

    new_s = [new_x, new_y, new_theta, new_v];


end

