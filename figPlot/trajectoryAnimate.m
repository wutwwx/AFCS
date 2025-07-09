function trajectoryAnimate(TarTra, SystemStates, N, EnvironStates, plotParas)
%TRAJECTORYANIMATE  Animate tracking performance for each ship in the formation.
%
%   trajectoryAnimate(TarTra, SystemStates, N, EnvironStates, plotParas)
%   animates (in separate figures for each ship) the target and actual trajectory
%   tracking process, overlaying static and dynamic obstacles as well as ship icons.
%
%   Inputs:
%     TarTra        - Cell array, TarTra{j} [N×3], target trajectory for each ship
%     SystemStates  - Cell array, SystemStates{j}.realStates [N×3], actual state for each ship
%     N             - Integer, total number of time steps to animate
%     EnvironStates - Struct, contains environment info (staticObs, manual_dynamic, etc.)
%     plotParas     - Struct, plotting parameters (colors, animateSpeed, animateInterval)
%
%   Usage Example:
%     trajectoryAnimate(MemTarTraSave,SystemStates,N,EnvironStates,plotParas)
%
%   Author: Wenxiang Wu (with ChatGPT enhancement)
%   Date:   2025-06-22

animateSpeed = plotParas.animateSpeed;
colors = plotParas.colors;
animateInterval = plotParas.animateInterval;
ShipNum = length(SystemStates);

% === Automatically compute plot boundaries, ignoring dynamic obstacles ===  
x_all = [];
y_all = [];
for j = 1:ShipNum
    x_all = [x_all; SystemStates{j}.realStates(1:N,1); TarTra{j}(1:N,1)];
    y_all = [y_all; SystemStates{j}.realStates(1:N,2); TarTra{j}(1:N,2)];
end
if isfield(EnvironStates, "staticObs")
    for k = 1:numel(EnvironStates.staticObs)
        x_all = [x_all; EnvironStates.staticObs{k}.Pos(:,1)];
        y_all = [y_all; EnvironStates.staticObs{k}.Pos(:,2)];
    end
end
margin = 5;
axisLimit.xmin = min(y_all) - margin;
axisLimit.xmax = max(y_all) + margin;
axisLimit.ymin = min(x_all) - margin;
axisLimit.ymax = max(x_all) + margin;

for j = 1:ShipNum
    xi{j} = SystemStates{j}.realStates;
    xd{j} = TarTra{j};
    figure; hold on; axis equal; box on; grid on;
    axis([axisLimit.xmin axisLimit.xmax axisLimit.ymin axisLimit.ymax]);
    xlabel('y (m)', 'FontName', 'Times New Roman');
    ylabel('x (m)', 'FontName', 'Times New Roman');
    set(gca, 'FontName', 'Times New Roman');

    % ==== 1. Static Obstacles ====  
    if isfield(EnvironStates, "staticObs")
        for k = 1:numel(EnvironStates.staticObs)
            obs = EnvironStates.staticObs{k};
            fill(obs.Pos(:,2), obs.Pos(:,1), colors.manObs, 'FaceAlpha', .5, 'EdgeColor', 'k', 'HandleVisibility','off');
        end
    end

    % ==== 2. Dynamic Obstacles ====  
    Ndyn = 0;
    if isfield(EnvironStates, "manual_dynamic") && isfield(EnvironStates.manual_dynamic, "TS") ...
            && isfield(colors, "dynObs") && ~isempty(colors.dynObs)
        dynShips = EnvironStates.manual_dynamic.TS;
        Ndyn = numel(dynShips);
        Tmax_dyn = size(dynShips{1}.Pos, 1);
    else
        Tmax_dyn = 0;
    end

    % ==== 3. Initialize Handles ====  
    h_tar = plot(nan, nan, 'k-', 'LineWidth', 2, 'DisplayName', 'Target Trajectory');
    h_real = plot(nan, nan, '--', 'Color', colors.ship{j}, 'LineWidth', 2, 'DisplayName', 'Actual Trajectory');
    h_ship = [];
    h_dyn = gobjects(Ndyn, 1);
    h_dyntraj = gobjects(Ndyn, 1);
    for k = 1:Ndyn
        h_dyntraj(k) = plot(nan, nan, ':', 'Color', colors.dynObs(k,:), 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Dynamic Obstacle %d', k));
    end

    % ==== 4. Main Animation Loop ====  
    maxStep = min(N, Tmax_dyn); if Tmax_dyn == 0, maxStep = N; end
    for t = 1:animateInterval:maxStep
        title({['Ship ', num2str(j), ' Tracking Animation'], ...
               ['Step: ', num2str(t), ' / ', num2str(maxStep)]});
        % Target Trajectory  
        set(h_tar, 'XData', xd{j}(1:t,2), 'YData', xd{j}(1:t,1));
        % Actual Trajectory  
        set(h_real, 'XData', xi{j}(1:t,2), 'YData', xi{j}(1:t,1));
        % Ship Icons  
        if isgraphics(h_ship), delete(h_ship); end
        psi = xi{j}(t,3);
        h_ship = shipDisplay3([psi 0 0], xi{j}(t,2), xi{j}(t,1), 0, 0.7, colors.ship{j});

        % Dynamic Obstacles
        for k = 1:Ndyn
            t_dyn = min(t, size(dynShips{k}.Pos,1));
            xHist = dynShips{k}.Pos(1:t_dyn,2);
            yHist = dynShips{k}.Pos(1:t_dyn,1);
            set(h_dyntraj(k), 'XData', xHist, 'YData', yHist);
            if isgraphics(h_dyn(k)), delete(h_dyn(k)); end
            psi_dyn = dynShips{k}.Hdg(t_dyn);
            h_dyn(k) = shipDisplay3([psi_dyn 0 0], dynShips{k}.Pos(t_dyn,2), dynShips{k}.Pos(t_dyn,1), 0, 0.7, colors.dynObs(k,:));
        end

        pause(animateSpeed); drawnow;
    end

    legend([h_tar, h_real, h_dyntraj(:)'], 'Location', 'Best');
end
end
