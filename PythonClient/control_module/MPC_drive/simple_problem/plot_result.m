function [] = plot_result(x, traj, opt_stat, lb, ub, circ_constraint)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
    fval = opt_stat(1);
    feas = opt_stat(2);
    figure('units', 'normalized','pos', [0.1526 0.4574 0.7823 0.3861])
    
    % Plot reference and solution
    subplot(2,3,[1,4])
    plot(x(:,3),x(:,4),'bo-', 'MarkerFaceColor', 'b', 'MarkerSize', 8)
    hold on
    plot(traj(:,1), traj(:,2),'go','MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g')
    title("Loss: " + num2str(fval) + "   Feasibility: " + num2str(sum(feas)))
    axis square
    grid minor
    xlabel x
    ylabel y
    
    % If using circular constraint as obstacle, plot it
    if nargin > 5
        axis([0 10 0 10]);
        fimplicit(circ_constraint);
    end

    % Plot states
    subplot(2,3,[2,3]);
    plot(x(:,5),'bo-', 'MarkerFaceColor', 'b');
    hold on;
    plot(x(:,6),'ro-', 'MarkerFaceColor', 'r');
    legend '\theta' v AutoUpdate off
    grid minor
    title States
    xlabel 'time step'
    plot(lb(:,5),'b--')
    plot(ub(:,5),'b--')
    plot(lb(:,6),'r--')
    plot(ub(:,6),'r--')

    % Plot inputs
    subplot(2,3,[5,6]);
    plot(x(:,1),'bo-', 'MarkerFaceColor', 'b');
    hold on;
    plot(x(:,2),'ro-', 'MarkerFaceColor', 'r');
    legend 'u_\theta' u_v AutoUpdate off
    grid minor
    title Inputs
    xlabel 'time step'
    plot(lb(:,1),'b--')
    plot(ub(:,1),'b--')
    plot(lb(:,2),'r--')
    plot(ub(:,2),'r--')

end

