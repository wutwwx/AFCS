function formationPlot(ForTra, SystemStates, N, EnvironStates, plotParas)
%FORMATIONPLOT   Plot ship formation trajectories and obstacles in a 2D scene.
%
%   formationPlot(ForTra, SystemStates, N, EnvironStates, plotParas) visualizes
%   the target and actual trajectories of a multi-ship formation, along with 
%   static and dynamic obstacles, using automatic axis scaling.
%
%   Inputs:
%     ForTra        - [N×3] or [N×2] target trajectory for the leader/reference (e.g., [x, y, psi])
%     SystemStates  - Cell array, each cell contains .realStates for each ship [N×3 or N×6]
%     N             - Number of time steps to plot (trajectory length)
%     EnvironStates - Struct, must contain at least fields:
%                       .staticObs: cell array of obstacles (each with .Pos)
%                       .manual_dynamic: struct with .TS dynamic obstacle (optional)
%     plotParas     - Struct of plotting parameters:
%                       .colors: color settings for ships, obstacles, etc.
%                       .iconInterval: spacing between ship icons on trajectory
%
%   Functionality:
%     - Automatically determines axis range based on all ship trajectories and obstacles.
%     - Plots static obstacles as shaded polygons (gray by default).
%     - Plots dynamic obstacles as colored lines/ships if present.
%     - Shows target (reference) trajectory as a solid line.
%     - Shows each ship's trajectory as dashed lines and ship icons along the path.
%     - Draws connection lines between ships (for visualization of formation structure).
%     - Adds legend for all elements and titles.
%
%   Example usage:
%       formationPlot(ForTarTraSave,SystemStates,N, EnvironStates,plotParas);
%
%   Author: Wenxiang Wu (with ChatGPT enhancement)
%   Date:   2025-06-20

colors = plotParas.colors;
iconInterval = plotParas.iconInterval;
ShipNum = length(SystemStates);
xi = cell(1, ShipNum);

for j = 1:ShipNum
    xi{j} = SystemStates{j}.realStates;
end

% === Automatically set axis range: ignore dynamic obstacles ===
allX = []; allY = [];
for j = 1:ShipNum
    allX = [allX; xi{j}(1:N,2)];
    allY = [allY; xi{j}(1:N,1)];
end
allX = [allX; ForTra(1:N,2)];
allY = [allY; ForTra(1:N,1)];
if isfield(EnvironStates, "staticObs")
    for k = 1:numel(EnvironStates.staticObs)
        obs = EnvironStates.staticObs{k};
        allX = [allX; obs.Pos(:,2)];
        allY = [allY; obs.Pos(:,1)];
    end
end
xrange = max(allX) - min(allX);
yrange = max(allY) - min(allY);
padX = 0.1 * xrange;
padY = 0.1 * yrange;
axisLimit.xmin = min(allX) - padX;
axisLimit.xmax = max(allX) + padX;
axisLimit.ymin = min(allY) - padY;
axisLimit.ymax = max(allY) + padY;

figure
hold on

% ==== 1. Static Obstacles ====
if isfield(EnvironStates, "staticObs")
    for k = 1:numel(EnvironStates.staticObs)
        obs = EnvironStates.staticObs{k};
        fill(obs.Pos(:,2), obs.Pos(:,1), colors.manObs, 'FaceAlpha', .5, 'EdgeColor', 'k');
    end
end

% ==== 2. Dynamic Obstacle Trajectories and Positions (visualized but do not affect axes) ====
dyn_plot_handles = [];
if isfield(EnvironStates, "manual_dynamic") && isfield(EnvironStates.manual_dynamic, "TS")
    dynShips = EnvironStates.manual_dynamic.TS;
    dynColors = colors.dynObs;
    for k = 1:numel(dynShips)
        dyn_traj = dynShips{k}.Pos;
        htraj = plot(dyn_traj(:,2), dyn_traj(:,1), ':', 'Color', dynColors(k,:), 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Dynamic Obstacle %d Trajectory', k));
        dyn_plot_handles = [dyn_plot_handles htraj];
        psi = dynShips{k}.Hdg(end);
        shipDisplay3([psi 0 0], dyn_traj(end,2), dyn_traj(end,1), 0, 0.7, dynColors(k,:));
    end
end

% ==== 3. Plot Target Trajectory ====
p{1} = plot(ForTra(1:N,2), ForTra(1:N,1), 'Color', colors.shipTra, 'LineWidth', 2, 'DisplayName', 'Target Trajectory');

% ==== 4. Plot Each Ship's Trajectory ====
for j = 1:ShipNum
    p{j+1} = plot(xi{j}(1:N,2), xi{j}(1:N,1), '--','Color', colors.ship{j}, 'LineWidth', 2, ...
        'DisplayName', sprintf('Ship %d Trajectory', j));
end

% ==== 5. Draw Connection Lines ====
for i = 1:floor(N/iconInterval)
    for j = 1:ShipNum
        X(i,j) = xi{j}(1+(i-1)*iconInterval,2);
        Y(i,j) = xi{j}(1+(i-1)*iconInterval,1);
    end
    for j = 1:ShipNum-1
        if j==1
            for t = 2:ShipNum
                plot([X(i,1), X(i,t)], [Y(i,1), Y(i,t)], '-.', 'Color', colors.connect, 'LineWidth', 1);
            end
        else
            plot([X(i,j), X(i,j+1)], [Y(i,j), Y(i,j+1)], '-.', 'Color', colors.connect, 'LineWidth', 1);
        end
    end
end

for j = 1:ShipNum
    X(floor(N/iconInterval)+1,j) = xi{j}(N,2);
    Y(floor(N/iconInterval)+1,j) = xi{j}(N,1);
end
for j = 1:ShipNum-1
    if j==1
        for t = 2:ShipNum
            plot([X(end,1), X(end,t)], [Y(end,1), Y(end,t)], '-.', 'Color', colors.connect, 'LineWidth', 1);
        end
    else
        plot([X(end,j), X(end,j+1)], [Y(end,j), Y(end,j+1)], '-.', 'Color', colors.connect, 'LineWidth', 1);
    end
end

for j = 1:ShipNum
    for i = 1:floor(N/iconInterval)
        shipDisplay3([xi{j}(1+(i-1)*iconInterval,3),0,0],xi{j}(1+(i-1)*iconInterval,2),xi{j}(1+(i-1)*iconInterval,1),[],[],colors.ship{j});
    end
    shipDisplay3([xi{j}(N,3),0,0],xi{j}(N,2),xi{j}(N,1),[],[],colors.ship{j});
end

axis([axisLimit.xmin axisLimit.xmax axisLimit.ymin axisLimit.ymax]);
axis equal;
xlabel('y (m)', 'FontName', 'Times New Roman');
ylabel('x (m)', 'FontName', 'Times New Roman');
set(gca, 'FontName', 'Times New Roman');
labels = arrayfun(@(j) ['Ship ' num2str(j) ' Trajectory'], 1:ShipNum, 'UniformOutput', false);
legend([p{:}, dyn_plot_handles], [{'Target Trajectory'}, labels, ...
    arrayfun(@(k) sprintf('Dynamic Obstacle %d Trajectory', k), 1:numel(dyn_plot_handles), 'UniformOutput', false)]);
title('Formation Trajectory');
end
