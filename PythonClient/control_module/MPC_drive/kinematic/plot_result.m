function [] = plot_result(x, traj, opt_stat, lb, ub, circ_constraint, f)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
    % Default figure is the current one
    if nargin < 7
        f = gcf;
    end

    fval = opt_stat(1);
    feas = opt_stat(2);
    set(f, 'units', 'normalized','pos', [0.1526 0.4574 0.7823 0.3861])
    
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
    
    
    first_color = [0, 0.4470, 0.7410];
    second_color = [0.8500, 0.3250, 0.0980];
    % Plot states    
    subplot(2,3,[2,3]);
    yyaxis left; hold on;
    plot(x(:,5),'o-', 'MarkerFaceColor', first_color);
    
    yyaxis right; hold on;
    plot(x(:,6),'o-', 'MarkerFaceColor', second_color);

    legend '\theta' v AutoUpdate off
    grid minor
    title States
    xlabel 'time step'
    
    yyaxis left
    plot(lb(:,5),'--', 'Color', first_color)
    plot(ub(:,5),'--', 'Color', first_color)
    
    yyaxis right
    plot(lb(:,6),'--', 'Color', second_color)
    plot(ub(:,6),'--', 'Color', second_color)
    
    % Plot inputs
    subplot(2,3,[5,6]);
    yyaxis left; hold on;
    plot(x(:,1),'o-', 'MarkerFaceColor', first_color);
    
    yyaxis right; hold on
    plot(x(:,2),'o-', 'MarkerFaceColor', second_color);
    
    legend 'u_\theta' u_v AutoUpdate off
    grid minor
    title Inputs
    xlabel 'time step'
    
    yyaxis left
    plot(lb(:,1),'--', 'Color', first_color)
    plot(ub(:,1),'--', 'Color', first_color)
    
    yyaxis right
    plot(lb(:,2),'--', 'Color', second_color)
    plot(ub(:,2),'--', 'Color', second_color)

end

