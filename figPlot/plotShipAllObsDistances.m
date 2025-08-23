function plotASVAllObsDistances(SystemStates, N, Ts, EnvironStates)
%PLOTASVALLOBSDISTANCES  Plot time-varying distances between each ASV and all obstacles.
%
%   plotASVAllObsDistances(SystemStates, N, Ts, EnvironStates)
%   computes and plots the distance from each ASV to all static and dynamic
%   obstacles at every time step, showing up to 4 ASVs per figure. For each 
%   ASV, the minimum distance to each obstacle is also printed in the command window.
%
%   Inputs:
%     SystemStates   - Cell array, SystemStates{j}.realStates [N×3], trajectory of each ASV
%     N              - Integer, number of time steps
%     Ts             - Scalar, simulation time step (s)
%     EnvironStates  - Struct, environment info (staticObs, manual_dynamic, etc.)
%
%   Outputs:
%     (None. Creates figure windows with distance-time plots and prints minimum values)
%
%   Usage Example:
%     plotASVAllObsDistances(SystemStates, N, Ts, EnvironStates)
%
%   Author: Wenxiang Wu (with ChatGPT enhancement)
%   Date:   2025-06-21

% Distance plots between each ASV and static/dynamic obstacles, max 4 ASVs per figure  
ASVNum = length(SystemStates);
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

for iASV = 1:ASVNum
    ASVTraj = SystemStates{iASV}.realStates(1:N,1:2); % N × 2
    minDistStatic = inf(numel(staticObs),1);
    minDistDyn    = inf(numel(dynObs),1);

    % Create new figure window (switch to a new figure every 4 ASVs)  
    if mod(iASV-1, 4) == 0
        figure('Name','ASV to Obstacles Distance'); 
    end
    subplot(2,2,mod(iASV-1,4)+1); hold on; box on;
    set(gca,'FontName','Times New Roman');

    % ==== Static Obstacles ====  
    for k = 1:numel(staticObs)
        obs = staticObs{k};
        if isfield(obs, 'center') && isfield(obs, 'Rad')
            dist = sqrt(sum((ASVTraj - obs.center).^2,2)) - obs.Rad;
        elseif isfield(obs, 'Pos')
            dist = min(pdist2(ASVTraj, obs.Pos),[],2);
        else
            continue;
        end
        plot(timeVec, dist, '-', 'LineWidth', 1.4, 'DisplayName', sprintf('StaticObs%d', k));
        minDistStatic(k) = min(dist);
    end

    % ==== Dynamic Obstacles ====   
    for k = 1:numel(dynObs)
        obsTraj = dynObs{k}.Pos(1:N,1:2); % N × 2
        dist = sqrt(sum((ASVTraj - obsTraj).^2,2));
        if isfield(EnvironStates.manual_dynamic,'R') && length(EnvironStates.manual_dynamic.R) >= k
            dist = dist - EnvironStates.manual_dynamic.R(k);
        end
        plot(timeVec, dist, '--', 'LineWidth', 1.4, 'DisplayName', sprintf('DynObs%d', k));
        minDistDyn(k) = min(dist);
    end

    xlabel('Time (s)','FontName','Times New Roman');
    ylabel('Distance (m)','FontName','Times New Roman');
    title(sprintf('ASV %d', iASV), 'FontName','Times New Roman');
    legend show;
    grid on;

    % Print minimum distance 
    fprintf('--- ASV %d ---\n', iASV);
    for k = 1:numel(staticObs)
        fprintf('  Min distance to StaticObs %d: %.3f m\n', k, minDistStatic(k));
    end
    for k = 1:numel(dynObs)
        fprintf('  Min distance to DynObs %d:    %.3f m\n', k, minDistDyn(k));
    end
end
end
