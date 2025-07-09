function plotShipAllObsDistances(SystemStates, N, Ts, EnvironStates)
%PLOTSHIPALLOBSDISTANCES  Plot time-varying distances between each ship and all obstacles.
%
%   plotShipAllObsDistances(SystemStates, N, Ts, EnvironStates)
%   computes and plots the distance from each ship to all static and dynamic
%   obstacles at every time step, showing up to 4 ships per figure. For each 
%   ship, the minimum distance to each obstacle is also printed in the command window.
%
%   Inputs:
%     SystemStates   - Cell array, SystemStates{j}.realStates [N×3], trajectory of each ship
%     N              - Integer, number of time steps
%     Ts             - Scalar, simulation time step (s)
%     EnvironStates  - Struct, environment info (staticObs, manual_dynamic, etc.)
%
%   Outputs:
%     (None. Creates figure windows with distance-time plots and prints minimum values)
%
%   Usage Example:
%     plotShipAllObsDistances(SystemStates, N, Ts, EnvironStates)
%
%   Author: Wenxiang Wu (with ChatGPT enhancement)
%   Date:   2025-06-21

% Distance plots between each ship and static/dynamic obstacles, max 4 ships per figure  
ShipNum = length(SystemStates);
timeVec = (0:N-1)*Ts;

% --------- Handle Static Obstacles ---------  
staticObs = {};
if isfield(EnvironStates,"staticObs") && ~isempty(EnvironStates.staticObs)
    staticObs = EnvironStates.staticObs;
end

% --------- Handle Dynamic Obstacles ---------  
dynObs = {};
if isfield(EnvironStates,"manual_dynamic") && isfield(EnvironStates.manual_dynamic,"TS") ...
        && ~isempty(EnvironStates.manual_dynamic.TS)
    dynObs = EnvironStates.manual_dynamic.TS;
end

for iShip = 1:ShipNum
    shipTraj = SystemStates{iShip}.realStates(1:N,1:2); % N × 2
    minDistStatic = inf(numel(staticObs),1);
    minDistDyn    = inf(numel(dynObs),1);

    % Create new figure window (switch to a new figure every 4 ships)  
    if mod(iShip-1, 4) == 0
        figure('Name','Ship to Obstacles Distance'); 
    end
    subplot(2,2,mod(iShip-1,4)+1); hold on; box on;
    set(gca,'FontName','Times New Roman');

    % ==== Static Obstacles ====  
    for k = 1:numel(staticObs)
        obs = staticObs{k};
        if isfield(obs, 'center') && isfield(obs, 'Rad')
            dist = sqrt(sum((shipTraj - obs.center).^2,2)) - obs.Rad;
        elseif isfield(obs, 'Pos')
            dist = min(pdist2(shipTraj, obs.Pos),[],2);
        else
            continue;
        end
        plot(timeVec, dist, '-', 'LineWidth', 1.4, 'DisplayName', sprintf('StaticObs%d', k));
        minDistStatic(k) = min(dist);
    end

    % ==== Dynamic Obstacles ====   
    for k = 1:numel(dynObs)
        obsTraj = dynObs{k}.Pos(1:N,1:2); % N × 2
        dist = sqrt(sum((shipTraj - obsTraj).^2,2));
        if isfield(EnvironStates.manual_dynamic,'R') && length(EnvironStates.manual_dynamic.R) >= k
            dist = dist - EnvironStates.manual_dynamic.R(k);
        end
        plot(timeVec, dist, '--', 'LineWidth', 1.4, 'DisplayName', sprintf('DynObs%d', k));
        minDistDyn(k) = min(dist);
    end

    xlabel('Time (s)','FontName','Times New Roman');
    ylabel('Distance (m)','FontName','Times New Roman');
    title(sprintf('Ship %d', iShip), 'FontName','Times New Roman');
    legend show;
    grid on;

    % Print minimum distance 
    fprintf('--- Ship %d ---\n', iShip);
    for k = 1:numel(staticObs)
        fprintf('  Min distance to StaticObs %d: %.3f m\n', k, minDistStatic(k));
    end
    for k = 1:numel(dynObs)
        fprintf('  Min distance to DynObs %d:    %.3f m\n', k, minDistDyn(k));
    end
end
end
