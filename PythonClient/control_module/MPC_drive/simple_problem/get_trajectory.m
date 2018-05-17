function [traj] = get_trajectory(f, limits)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    if nargin < 2
        limits = [0 1 0 1];
    end

    figure(f)
    axis(limits)
    title('Click to add points. Press enter when finished.')
    pbaspect([1 1 1])
    grid on
    
    [x,y] = getpts();
    traj = [x y];    
    
    close(f)

end

