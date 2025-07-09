function staticObstacle = staticObstacleRand(nav_area_points, avoid_pointsInfo)
%STATICOBSTACLERAND   Generate random polygonal static obstacles within a navigable area.
%
%   staticObstacle = staticObstacleRand(nav_area_points, avoid_pointsInfo)
%   randomly generates a set of static obstacles (regular polygons) for use 
%   in path planning, navigation, or collision avoidance simulations.
%
%   Inputs:
%     nav_area_points   : [n×2] matrix, vertices of the navigable area boundary polygon
%     avoid_pointsInfo  : [m×3] matrix, positions and radii of points to avoid
%                         (e.g., [x, y, r] for each avoid region)
%
%   Outputs:
%     staticObstacle    : Cell array (obsNum×1), each cell is a struct:
%                          - .points : [sides_num+1 × 2] obstacle polygon vertices ([x, y])
%                          - .center : [1×2] obstacle center position ([x, y])
%                          - .radius : Obstacle radius (scalar)
%
%   Description:
%     - Obstacles are placed randomly within the navigable area, ensuring
%       no overlap with avoid regions or previously placed obstacles.
%     - Each obstacle is a regular polygon (default: hexagon, can be changed by sides_num).
%     - Uses pdist2 to compute distances and guarantees non-overlapping placement.
%     - Edit obsNum, obsRange, sides_num in the code for custom scenarios.
%
%   Example usage:
%     nav_area_points = [0 0; 0 30; 30 30; 30 0];
%     avoid_pointsInfo = [15 15 3]; % Example: avoid circle at (15,15), r=3
%     staticObstacle = staticObstacleRand(nav_area_points, avoid_pointsInfo);
%
%   Author: Huimin Chen
%   Date:   2024-02-05

    % Parameters
    obsNum = 3;             % Number of obstacles to randomly generate
    obsRange = [0, 2]; % Range for the radius of the obstacles
    sides_num = 6;          % Number of sides for random obstacles (e.g., hexagons)

    % Initialize the output cell array
    staticObstacle = cell(obsNum, 1);

    % Store centers and radii of generated obstacles to prevent overlap
    existing_centers = [];
    existing_radii = [];

    % Generate obstacles
    for i = 1:obsNum
        radius = randi(obsRange); % Random radius for the obstacle

        % Randomly generate the center of the obstacle
        isValid = false;
        while ~isValid
            % Generate random coordinates within the navigable area
            rand_x = rand(1, 1) * (max(nav_area_points(:, 1)) - min(nav_area_points(:, 1)) - 2 * radius) + min(nav_area_points(:, 1)) + radius;
            rand_y = rand(1, 1) * (max(nav_area_points(:, 2)) - min(nav_area_points(:, 2)) - 2 * radius) + min(nav_area_points(:, 2)) + radius;

            % Check if the center is within the navigable area boundary
            if inpolygon(rand_x, rand_y, nav_area_points(:, 1), nav_area_points(:, 2))
                % Check distance from avoid points (including their radius)
                if ~isempty(avoid_pointsInfo)
                    avoid_points_centers = avoid_pointsInfo(:, 1:2);
                    avoid_points_radii = avoid_pointsInfo(:, 3);
                    avoid_points_distances = pdist2([rand_x, rand_y], avoid_points_centers) - avoid_points_radii';
                else
                    avoid_points_distances = inf; % No avoid points
                end

                % Check distance from existing obstacles to ensure no overlap
                if ~isempty(existing_centers)
                    existing_distances = pdist2([rand_x, rand_y], existing_centers) - (existing_radii + radius)';
                else
                    existing_distances = inf; % No existing obstacles
                end

                % Check all conditions: navigable area, avoid points, and no overlap with other obstacles
                if min(avoid_points_distances) >= radius && min(existing_distances) >= 0
                    isValid = true;
                end
            end
        end

        % Store the new obstacle's center and radius to prevent future overlaps
        existing_centers = [existing_centers; rand_x, rand_y];
        existing_radii = [existing_radii; radius];

        % Generate regular polygon vertices for the obstacle
        theta = linspace(0, 2 * pi, sides_num + 1)'; % Generate angles for the vertices
        x = radius * cos(theta) + rand_x;
        y = radius * sin(theta) + rand_y;

        % Create the obstacle and store it in the cell array
        staticObstacle{i} = struct();
        staticObstacle{i}.points = [x, y];           % Polygon vertices (n*2 matrix)
        staticObstacle{i}.center = [rand_x, rand_y]; % Obstacle center (1*2 vector)
        staticObstacle{i}.radius = radius;           % Obstacle radius (scalar)
    end
end
