function [c] = carla_objects_constraints(dMaps, x, y)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    
    n_objects = size(dMaps,3);
    n_time_steps = length(x);
    c = zeros(n_objects,n_time_steps);
    for i = 1:n_objects
        c(i,:) = -interpolate(dMaps(:,:,i), x, y);
    end
    c = c(:);

end

