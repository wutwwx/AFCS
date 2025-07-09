function sampled_points = resampleCurve(curve, interval)
%RESAMPLECURVE   Resample a 2D curve by arc length and calculate tangent angles.
%
%   sampled_points = resampleCurve(curve, interval)
%   Given a 2D path (Nx2), returns a sequence of points with approximately
%   constant arc-length spacing and computes the tangent angle at each sample.
%
%   Inputs:
%     curve     - [N x 2] array, each row [x, y] representing a polyline vertex
%     interval  - Scalar, desired sampling interval (in meters, or consistent units)
%
%   Outputs:
%     sampled_points - [M x 3] array, each row [x, y, theta] where theta is
%                      the tangent direction (radians) at the point
%
%   Features:
%     - Removes duplicate points to avoid interp1 errors
%     - Handles arbitrary polyline input, returns the last point exactly
%     - Tangent at each resampled point is the direction of the next segment
%
%   Example:
%     sampled = resampleCurve(curve, 0.5);  % Uniformly sample every 0.5 m
%
%   Author: Wenxiang Wu (with ChatGPT enhancement)
%   Date:   2025-03-06

    % === 1. Remove duplicate points (to avoid interp1 errors) ===  
    delta = diff(curve);
    distances = sqrt(sum(delta.^2, 2));
    valid_idx = [true; distances > 1e-10];  % Retain the first point and all non-duplicate points 
    curve = curve(valid_idx, :);

    % === 2. Compute cumulative arc length ===  
    distances = sqrt(sum(diff(curve).^2, 2));
    cumulative_length = [0; cumsum(distances)];
    total_length = cumulative_length(end);

    % === 3. Construct target sampling arc-length sequence ===  
    n_points = floor(total_length / interval);
    target_lengths = (0:n_points) * interval;
    if total_length - target_lengths(end) > 1e-10
        target_lengths(end+1) = total_length;
    end

    % === 4. Interpolate to compute sampled point coordinates ===   
    x_interp = interp1(cumulative_length, curve(:,1), target_lengths, 'linear');
    y_interp = interp1(cumulative_length, curve(:,2), target_lengths, 'linear');
    sampled_xy = [x_interp', y_interp'];

    % === 5. Compute slope angle theta (unit: radians) ===  
    dx = diff(sampled_xy(:,1));
    dy = diff(sampled_xy(:,2));
    theta = atan2(dy, dx);
    theta(end+1) = theta(end); % Append direction angle for the last point  
    theta = theta(:);          % Convert to column vector  
    
    % === 6. Concatenate output === 
    sampled_points = [sampled_xy, theta];
end
