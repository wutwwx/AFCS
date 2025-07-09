function obstacle = staticObstacleManual(navArea)
%STATICOBSTACLEMANUAL   Generate circular static obstacles at predefined locations within a map.
%
%   obstacle = staticObstacleManual(navArea) produces a cell array of
%   obstacles, each defined as a set of points approximating a circle with 
%   specified center and radius. The scenario is set by the hard-coded 
%   obstacleInfo matrix within the function.
%
%   Inputs:
%     navArea   : [n×2] matrix specifying navigation area corners (e.g., rectangle)
%
%   Outputs:
%     obstacle  : Cell array (1×obstacleNum), each cell is a struct with fields:
%                   - .point  : [numPoints × 2] matrix, circle vertices [x, y]
%                   - .radius : Obstacle radius
%                   - .center : [1×2] obstacle center [x, y]
%
%   Description:
%     - Centers and radii of obstacles are defined in the obstacleInfo matrix.
%     - Each obstacle is represented as a regular polygon (circle approx.).
%     - The function checks that each obstacle is within the map bounds.
%     - Commented plotting code is provided for visualization.
%
%   Example:
%     navArea = [0 0; 0 30; 30 30; 30 0];
%     obstacle = staticObstacleManual(navArea);
%
%   Author: Huimin Chen
%   Date:   2025-02-01


    obstacleInfo = [
            13, 12, 2.5;  % First obstacle: center (13, 12), radius 2.5
            5, 5, 1;  % Second obstacle: center (5, 5), radius 1
            17, 22, 1.5   % Third obstacle: center (17, 22), radius 1.5
        ];

    % Number of obstacles
    obstacleNum = size(obstacleInfo, 1);

    % Initialize the cell array to hold obstacle structures
    obstacle = cell(1, obstacleNum);

    % Generate obstacles using predefined information
    for i = 1:obstacleNum
        % Extract center coordinates and radius for each obstacle
        center = obstacleInfo(i, 1:2);
        radius = obstacleInfo(i, 3);

        % Ensure the center and radius are valid
        if center(1) < navArea(1,1) || center(1) > navArea(3,1) || center(2) < navArea(1,2) || center(2) > navArea(3,2) || radius <= 0
            error('Invalid center or radius for obstacle %d. Ensure values are within map bounds and radius is positive.', i);
        end

        % Generate points for the circular obstacle (using 50 points to approximate the circle)
        numPoints = 50;  % Number of points to approximate the circle
        theta = linspace(0, 2*pi, numPoints);
        xPoints = center(1) + radius * cos(theta);
        yPoints = center(2) + radius * sin(theta);
        
        % Store the points in the obstacle cell array
        obstacle{i}.point = [xPoints', yPoints']; % Each row represents [x, y] coordinates of a vertex
        obstacle{i}.radius = radius;
        obstacle{i}.center = center;
    end

    % % Plot the obstacles on the map for visualization
    % figure;
    % hold on;
    % box on;
    % axis equal;
    % grid on;
    % axis([0, MapSize(1), 0, MapSize(2)]);
    % title('Generated Obstacles in Map');
    % xlabel('X Coordinate');
    % ylabel('Y Coordinate');
    % 
    % % Plot each obstacle
    % for i = 1:obstacleNum
    %     fill(obstacle{i}.point(:, 1), obstacle{i}.point(:, 2), 'k', 'FaceAlpha', 0.5, 'EdgeColor', 'k');
    % end
    % hold off;
end
