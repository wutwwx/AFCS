function [task] = taskManualSet()
% TASKMANUALSET   Define manual multi-stage formation tasks and geometry.
%
%   Output:
%     task - Struct containing task names, start times, geometric formation layout,
%            and path/velocity targets for each phase.
%
%   Usage:
%     Used to initialize ship formation scenarios (e.g., for formation MPC simulation).
%
%   Example:
%     task = taskManualSet();
%
%   Author: Wenxiang Wu
%   Date:   2025-06-19

% ------------------------------------------------------------------------
% 1. Load Predefined Reference Path
% ------------------------------------------------------------------------
load('targetpath', 'xr');  % 'xr' is Nx2 or Nx3: [x, y, (optional) angle]

% ------------------------------------------------------------------------
% 2. Define Task Sequence and Timing (Example: Cybership2)
% ------------------------------------------------------------------------
task.name = ["PathFollowing", "Reconfiguration"];   % Task stages (e.g. path following, formation reconfiguration)
task.time = [0, 160];                              % Start times for each stage (seconds, same units as simulation)
% Geometry: [row = task index, col = ship]
task.geometry.distances = [0, 6, 4.8, 6;           % Distance to virtual reference point for each ship
                           0, 6, 4.8, 6];
task.geometry.angles    = [0, (1+0.20483277)*pi, pi, (1-0.20483277)*pi; % Relative angle for each ship (radians)
                           0, (1-0.20483277)*pi, pi, (1+0.20483277)*pi];
task.target{1}.path = xr(:,1:2);                   % Reference path for the first task (Nx2, [x y])
task.target{1}.velocity = 0.2;                     % Reference velocity for the first stage

% % ----------------------------------------------------------------------
% % Alternative: Yunfan1 Example (commented)
% % ----------------------------------------------------------------------
% load('targetpath_2', 'xr');
% task.name = ["PathFollowing"];
% task.time = 0;
% task.geometry.distances = [0, 6, 4.8, 6];
% task.geometry.angles    = [0, (1+0.20483277)*pi, pi, (1-0.20483277)*pi];
% task.target{1}.path = xr(:,1:2);
% task.target{1}.velocity = 0.5;

% ------------------------------------------------------------------------
% 3. Dimension Consistency Checks
% ------------------------------------------------------------------------
if length(task.name) ~= length(task.time)
    error('The number of tasks does not equal the number of task start times.');
end

if size(task.geometry.distances, 1) == 1
    task.geometry.distances = repmat(task.geometry.distances, length(task.name), 1);
elseif length(task.name) ~= size(task.geometry.distances, 1)
    error('The number of tasks does not equal the number of distance rows in geometry.');
end

if size(task.geometry.angles, 1) == 1
    task.geometry.angles = repmat(task.geometry.angles, length(task.name), 1);
elseif length(task.name) ~= size(task.geometry.angles, 1)
    error('The number of tasks does not equal the number of angle rows in geometry.');
end

end
