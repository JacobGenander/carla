function [throttle, brake] = evaluate_net(net, state)
%EVALUATE_NET(net, state) computes predicted throttle and brake from state
%values measured by CARLA and desired value for acceleration.
%
%   NOTE: If the car is standing still, predicting the brake reliably is
%   impossible (you could use any braking from 0 to 1 when standing still
%   and there would be no observable changes in the measured states).
%
%--------------------------------------------------------------------------
%   Inputs:
%--------------------------------------------------------------------------
%   
%   - net: network object, loaded from net.mat file provided together with
%   this function.
%
%   - state: structure, containing the relevant parts of the measurements
%   provided by the CARLA simulator, at N different time-steps. Contains 
%   the following fields:
%
%       - a_v: Desired acceleration in the direction of the chassis of the 
%         vehicle (Nx1 vector).
%
%       - delta_f: Front wheel angle in radians (Nx1 vector).
%
%       - forward_speed (Nx1 vector).
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%
%   - throttle: Predicted throttle values (Nx1 vector).
%   
%   - brake: Predicted brake values (Nx1 vector).
%
%

    
    %% Evaluate network and return output
    % Evaluate network
    input = [state.a_v'; state.forward_speed'; state.forward_speed.^2'; state.delta_f'];
    output = net(input);

    % Clip output to range from 0 to 1
    output = max(0,min(1,output));
    
    % Split output in two vectors
    throttle = output(1,:);
    brake = output(2,:);
    
    
    %% Ad-hoc post-processing
    
    % Set throttle to zero if brake is sufficiently large
    throttle(brake > 0.1) = 0;   
    
    % Set brake to zero if throttle is sufficiently large
    brake(throttle > 0.1) = 0;
    

end

