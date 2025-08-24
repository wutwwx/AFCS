function dynStates = dynamicObstacle(Sim_step, period)
%DYNAMICOBSTACLE   Simulate and generate trajectories for dynamic obstacles (ASVs).
%
%   dynStates = dynamicObstacle(Sim_step, period) produces a cell array 
%   containing the trajectory, speed, and heading history for each simulated 
%   dynamic obstacle (e.g., other ASVs) over a specified simulation horizon.
%
%   Inputs:
%     Sim_step   : Number of simulation time steps
%     period     : Time interval per step (s)
%
%   Outputs:
%     dynStates  : Cell array (1×N_obstacles), each cell is a struct with fields:
%                   - path     : [Sim_step+1 × 2] array, obstacle position history [yEast, xNorth]
%                   - spd_hist : [Sim_step+1 × 1] speed history (m/s)
%                   - hdg_hist : [Sim_step+1 × 1] course/heading history (rad)
%                   - ASV_len : Scalar, ASV length (m)
%
%   Description:
%     - Each dynamic obstacle is initialized with speed, course, and position.
%     - The trajectory is updated at each step assuming constant control input (here: constant speed/course).
%     - Uses the classASV object for state management and history recording.
%     - Trajectories can be used for scenario-based path planning, collision avoidance, and multi-agent simulation.
%
%   Author: Huimin Chen
%   Date:   2025-02-01

    % Example ASV configurations
    ASVNames = {'ASV1','ASV2'};
    initStates = {
        [0.3, 3/4*pi, 1.5, 30, 0];
        [0.25, -1/4*pi, 1.5, 0, 20];

    };
    ctrls=cell(length(initStates),1);
    for j=1:length(initStates)
        ctrls{j,1} = struct('newSpeed',initStates{j}(1,1), 'newCourse',initStates{j}(1,2));
    end
    N = numel(ASVNames);

    % Preallocate history storage
    ASV     = cell(1,N);
    paths    = cell(1,N);
    spdHist  = cell(1,N);
    hdgHist  = cell(1,N);
    dynStates = cell(1,N);
    ASVLen = cell(1,N);

    % Initialize histories: position, speed, and heading
    for i = 1:N
        ASV{i} = classASV(initStates{i});
        paths{i}   = ASV{i}.position;   % initial position
        spdHist{i} = ASV{i}.speed;      % initial speed
        hdgHist{i} = ASV{i}.course;     % initial heading
        ASVLen{i} = ASV{i}.length;     % initial length
    end

    % Loop Sim_step times, each step duration = period
    for t = 1:Sim_step
        for i = 1:N
            % Update current speed and course
            ASV{i}.speed  = ctrls{i}.newSpeed;
            ASV{i}.course = ctrls{i}.newCourse;
            % Convert course to MATLAB coordinate angle and compute displacement
            theta = ASV{i}.course;
            dpos = [cos(theta), sin(theta)] * ASV{i}.speed * period;
            ASV{i}.position = ASV{i}.position + dpos;
            % Append new state to history arrays
            paths{i}(end+1, :)   = ASV{i}.position;
            spdHist{i}(end+1, 1) = ASV{i}.speed;
            hdgHist{i}(end+1, 1) = ASV{i}.course;
        end
    end

    % Build output structure array
    for i = 1:N
        dynStates{i} = struct(...
            'path',     paths{i}, ...
            'spd_hist', spdHist{i}, ...
            'hdg_hist', hdgHist{i}, ...
            'ASV_len', ASVLen{i} ...
        );
    end
end

