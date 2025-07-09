clear all
close all

% =========================================================================
% Generate a Custom U-Shaped Path for Multi-Ship Formation Task
% =========================================================================
% This script creates a composite path consisting of:
%   - A straight line segment
%   - A semicircular arc
%   - Another straight line segment
% The output is a Nx2 array (xr), where each row is [x, y] along the path.
% The result is visualized and saved as 'targetpath.mat' for later use.
%
% Author: Wenxiang Wu
% Date:   2025-01-19


x0 = 5; y0 = 15;     % Starting point
x1 = 13; y1 = 15;    % Endpoint of the first straight segment, also start of the semicircle
x2 = 13; y2 = 25;    % Endpoint of the semicircle, also start of the third straight segment
x3 = -10;  y3 = 25;  % Endpoint
R = 5;               % Radius of the semicircle

% First straight segment
N1 = 50;
x_line1 = linspace(x0, x1, N1);
y_line1 = linspace(y0, y1, N1);

% Semicircular segment
Narc = 100;
center_x = x1; center_y = (y1+y2)/2;   % Circle center (13,20)
theta1 = -pi/2;       % Start from the lower tangent point
theta2 = pi/2;        % End at the upper tangent point
theta_arc = linspace(theta1, theta2, Narc);
x_arc = center_x + R * cos(theta_arc);
y_arc = center_y + R * sin(theta_arc);

% Third straight segment
N2 = 50;
x_line2 = linspace(x2, x3, N2);
y_line2 = linspace(y2, y3, N2);

% Combine segments
X = [x_line1, x_arc, x_line2];
Y = [y_line1, y_arc, y_line2];
xr=[X;Y]';

% Visualization
figure;
plot(Y, X, 'k-', 'LineWidth', 2); hold on;
axis equal
xlabel('Y'); ylabel('X'); title('Custom U-shaped Path Illustration');
legend('Overall Path')
grid on
  
axis equal

save('targetpath','xr')
