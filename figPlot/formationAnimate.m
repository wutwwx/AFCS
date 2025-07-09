function formationAnimate(ShipsTrajCell, N, EnvironStates, plotParas)
% formationAnimate   Animate multi-ship formation and obstacles (with static/dynamic plotting support).
%
%   formationAnimate(ShipsTrajCell, N, EnvironStates, plotParas) animates the
%   evolution of a ship formation and environmental obstacles over N time steps.
%
%   Inputs:
%     ShipsTrajCell   - Cell array, each element contains a ship's trajectory struct
%                       (should include field realStates: N×6 or N×3, [x, y, psi, ...])
%     N               - Number of animation steps (frames)
%     EnvironStates   - Struct with environment info (fields: .staticObs, .manual_dynamic)
%     plotParas       - Struct with plotting parameters:
%                         .colors: color settings for ships/obstacles
%                         .animateSpeed: pause duration per frame (s)
%                         .animateInterval: frame step interval
%
%   Visualization Features:
%     - Ships: trajectory and icon animation
%     - Static obstacles: shaded polygons/circles (gray by default)
%     - Dynamic obstacles: animated trajectory and icon (custom colors)
%     - Auto axis: range automatically fits all trajectories and obstacles
%     - Legend auto-generated for all ships and obstacles
%     - Step number displayed in figure title for animation tracking
%
%   Usage Example:
%       plotParas = plotParaSetting(EnvironStates, SystemStates, TarTra);
%       formationAnimate(SystemStates, N, EnvironStates,plotParas);
%
%   Author: Wenxiang Wu (with ChatGPT enhancement)
%   Date:   2025-06-20

animateSpeed     = plotParas.animateSpeed;
animateInterval  = plotParas.animateInterval;
colors           = plotParas.colors;
ShipNum          = numel(ShipsTrajCell);

% ==== Dynamic Obstacle Parsing ====
dynShips = {};
Ndyn = 0; Tmax_dyn = 0;
if isfield(EnvironStates, "manual_dynamic") && isfield(EnvironStates.manual_dynamic, "TS")
    dynShips = EnvironStates.manual_dynamic.TS;
    Ndyn = numel(dynShips);
    Tmax_dyn = size(dynShips{1}.Pos, 1);
end

% ==== Automatically compute axisLimit, ignoring dynamic obstacles ====
allX = []; allY = [];
for j = 1:ShipNum
    allX = [allX; ShipsTrajCell{j}.realStates(1:N,2)];
    allY = [allY; ShipsTrajCell{j}.realStates(1:N,1)];
end
if isfield(EnvironStates, "staticObs")
    for k = 1:numel(EnvironStates.staticObs)
        obs = EnvironStates.staticObs{k};
        allX = [allX; obs.Pos(:,2)];
        allY = [allY; obs.Pos(:,1)];
    end
end
xrange = max(allX) - min(allX);
yrange = max(allY) - min(allY);
padX = 0.1 * xrange; padY = 0.1 * yrange;
axisLimit.xmin = min(allX) - padX;
axisLimit.xmax = max(allX) + padX;
axisLimit.ymin = min(allY) - padY;
axisLimit.ymax = max(allY) + padY;

% ==== Trajectory Step Limitation ====
Tmax = N;
maxStep = min(Tmax, Tmax_dyn);
if Tmax_dyn == 0, maxStep = Tmax; end

figure; hold on; axis equal; box on; grid on;
axis([axisLimit.xmin axisLimit.xmax axisLimit.ymin axisLimit.ymax]);
xlabel('y (m)'); ylabel('x (m)');
title('Formation Animation');
set(gca, 'FontName', 'Times New Roman');

% ==== Static Obstacle Plotting ====
if isfield(EnvironStates, "staticObs")
    for k = 1:numel(EnvironStates.staticObs)
        obs = EnvironStates.staticObs{k};
        fill(obs.Pos(:,2), obs.Pos(:,1), colors.manObs, 'FaceAlpha', .4, 'EdgeColor', 'k', 'HandleVisibility','off');
    end
end

% ==== Object Initialization ====
trajHandles = gobjects(ShipNum,1);
shipHandles = gobjects(ShipNum,1);
for j = 1:ShipNum
    trajHandles(j) = plot(nan, nan, '--', 'Color', colors.ship{j}, 'LineWidth', 1.8);
end

dynShipHandles = gobjects(Ndyn,1);
dynTrajHandles = gobjects(Ndyn,1);
for k = 1:Ndyn
    dynTrajHandles(k) = plot(nan, nan, ':', 'Color', colors.dynObs(k,:), 'LineWidth', 1.3);
end

% ==== Main Animation Loop ====
for t = 1:animateInterval:maxStep
    title({['Formation Animation'], ['Step: ', num2str(t), ' / ', num2str(maxStep)]});
    for j = 1:ShipNum
        hisX = ShipsTrajCell{j}.realStates(1:t,2);
        hisY = ShipsTrajCell{j}.realStates(1:t,1);
        set(trajHandles(j), 'XData', hisX, 'YData', hisY);
        if isgraphics(shipHandles(j)), delete(shipHandles(j)); end
        psi = ShipsTrajCell{j}.realStates(t,3);
        shipHandles(j) = shipDisplay3([psi 0 0], ShipsTrajCell{j}.realStates(t,2), ShipsTrajCell{j}.realStates(t,1), 0, 0.7, colors.ship{j});
    end

    for k = 1:Ndyn
        t_dyn = min(t, size(dynShips{k}.Pos,1));
        xHist = dynShips{k}.Pos(1:t_dyn,2);
        yHist = dynShips{k}.Pos(1:t_dyn,1);
        set(dynTrajHandles(k), 'XData', xHist, 'YData', yHist);
        if isgraphics(dynShipHandles(k)), delete(dynShipHandles(k)); end
        psi_dyn = dynShips{k}.Hdg(t_dyn);
        dynShipHandles(k) = shipDisplay3([psi_dyn 0 0], dynShips{k}.Pos(t_dyn,2), dynShips{k}.Pos(t_dyn,1), 0, 0.7, colors.dynObs(k,:));
    end
    pause(animateSpeed); drawnow;
end

% ==== Automatic Legend Generation ====
labelShips = arrayfun(@(j) sprintf('Ship %d', j), 1:ShipNum, 'UniformOutput', false);
labelDyn = arrayfun(@(k) sprintf('Dynamic Obs %d', k), 1:Ndyn, 'UniformOutput', false);
legend([trajHandles; dynTrajHandles], [labelShips, labelDyn]);

end
