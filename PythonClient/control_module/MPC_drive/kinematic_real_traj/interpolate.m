function [interp] = interpolate(V, xQuery, yQuery)
    % INTERPOLATE Returns the linearly interpolated value val in 
    % the query point (qx, qy), together with the gradient (dx, dy).
    % V defines a grid where each value in V corresponds to the center
    % value in each grid cell. The value in V(1,1) is thus the value 
    % corresponding to coordinates (0.5, 0.5). 
    % 
    % THIS NEEDS TO BE UPDATED. NOW QUERY POINTS ARE SUPPOSED TO BE
    % SUPPLIED IN WORLD COORDINATES, NOT MAP COORDINATES.
    %
    % Query points which are less
    % than 0.5 from V's boundaries (e.g. (Xquery=0.3,Yquery=6.0)) will not
    % be interpolated in the bound dimension (e.g. no interpolation in 
    % x-dimension but in y-dimension). The partial derivative of the 
    % dimension is then set to either Inf or -Inf. V has (0,0) in the upper 
    % left corner and the positive direction of axes are right and 
    % downwards.
    
    nPoints = length(xQuery);
    if (nPoints ~= length(yQuery))
        error('Xquery and Yquery must have equal length')
    end
    
    % Convert from world coordinates [m] to map coordinates [px].
    xQuery = (xQuery+16.43)/0.1643;
    yQuery = (yQuery+16.43)/0.1643;

    % Four pixels are of relevance, defined below
    % coordinates:          values:
    % (x1,y1) (x2,y1)       (v1) (v2)
    % (x1,y2) (x2,y2)       (v3) (v3)
    
    nPoints = length(xQuery);
    interp = zeros(nPoints,3); % Holds interpolated values and gradients
    height = size(V,1);
    width = size(V,2);
    
    % For each query point
    for iPoint = 1:nPoints
        
        % Get the query point coordinates
        x = xQuery(iPoint);
        y = yQuery(iPoint);
        
        % Check if query point lies outside value grid
        if x > width || y > height || x < 0 || y < 0
            error('Query point is out of bounds')
        end

        boundL = x < 0.5;               % Are we at left boundary?
        boundR = x > width - 0.5;   % Are we at right boundary?
        boundT = y < 0.5;               % Are we at upper boundary?
        boundB = y > height - 0.5;   % Are we att lower boundary?
        
        % Get the pixel index which the query point lies in.
        % Here we assume pixel indices start at 0.
        if boundR
            xp = width - 1;
        else
            xp = floor(x);
        end
        
        if boundB
            yp = height - 1;
        else
            yp = floor(y);
        end
        
        decimalX = x - xp; % This is the decimal part of x
        decimalY = y - yp; % This is the decimal part of y
        
        if decimalX < 0.5
            x1 = xp - 1;   % Set x1 to the be the pixel to the left of x
            x2 = xp;       % The query point is in pixel x2
        else
            x1 = xp;       % We're interested in the pixel to the right of x
            x2 = xp + 1;   % Set x2 to the be the pixel index of x
        end
        
        if decimalY < 0.5
            y1 = yp - 1;   
            y2 = yp;
        else
            y1 = yp;       
            y2 = yp + 1;
        end
        
        % Find which plane the point lies in. Planes are numbered this way:
        % _____
        %|\ 2 /|
        %|3\ /1|
        %| / \ |
        %|/_4_\|
        
        cX = x - x2;
        cY = y - y2;
        
        if cX >= 0 && cX >= abs(cY)
            plane = 1;
        elseif cX < 0 && abs(cX) >= abs(cY)
            plane = 3;
        elseif cY >= 0 && cY >= abs(cX)
            plane = 4;
        else
            plane = 2;
        end

        % The most common case is to not be on any boundary, so start there
        if ~(boundL || boundR || boundT || boundB)
            
            % Get value in each pixel of relevance
            v1 = V(y1 + 1,x1 + 1);
            v2 = V(y1 + 1,x2 + 1);
            v3 = V(y2 + 1,x1 + 1);
            v4 = V(y2 + 1,x2 + 1);

            a = v3*((x2+0.5) - x) + v4*(x - (x1+0.5));
            b = v1*((x2+0.5) - x) + v2*(x - (x1+0.5));
            c = b*((y2+0.5) - y) + a*(y - (y1+0.5));
            
            % Caluclate value in origo of the four pixels
            o = (v1 + v2 + v3 + v4) / 4;
            
            % Calculate the gradient
            if plane == 1
                dy = v4 - v2;
                dx = ((v4 + v2)/2 - o) * 2;
            elseif plane == 2
                dx = v2 - v1;
                dy = (o - (v2 + v1)/2) * 2;
            elseif plane == 3
                dy = v3 - v1;
                dx = (o - (v3 + v1)/2) * 2;
            else
                dx = v4 - v3;
                dy = ((v4 + v3)/2 - o) * 2;
            end
    
        % If we are at a corner boundary
        elseif (boundL || boundR) && (boundT || boundB)
            
            % Just return pixel value without interpolating
            c = V(yp + 1,xp + 1);
            
            if boundL 
                dx = -Inf;
            else
                dx = Inf;
            end
            
            if boundT
                dy = -Inf;
            else
                dy = Inf;
            end
         
        elseif boundT % If we are at top boundary
            
            % Interpolate in x only
            v3 = V(y2 + 1,x1 + 1);
            v4 = V(y2 + 1,x2 + 1);
            c = v3*((x2+0.5) - x) + v4*(x - (x1+0.5));
            dx = v4 - v3;
            dy = -Inf;
            
        elseif boundB % If we are at bottom boundary
            
            % Interpolate in x only
            v1 = V(y1 + 1,x1 + 1);
            v2 = V(y1 + 1,x2 + 1);
            c = v1*((x2+0.5) - x) + v2*(x - (x1+0.5));
            dx = v2 - v1;
            dy = Inf;

        elseif boundL % If we are at left boundary
            
            % Interpolate in y only
            v2 = V(y1 + 1,x2 + 1);
            v4 = V(y2 + 1,x2 + 1);
            c = v2*((y2+0.5) - y) + v4*(y - (y1+0.5));
            dy = v4 - v2;
            dx = -Inf;
                
        else % We must be at right boundary
            
            % Interpolate in y only
            v1 = V(y1 + 1,x1 + 1);
            v3 = V(y2 + 1,x1 + 1);
            c = v1*((y2+0.5) - y) + v3*(y - (y1+0.5));
            dy = v3 - v1;
            dx = Inf;
            
        end
        
        interp(iPoint,:) = [c, dx, dy];

    end
    
    dx = interp(:,2);
    dy = interp(:,3);
    interp = interp(:,1);
    
end