function trajectoryPlot(TarTra, SystemStates, N, EnvironStates, plotParas)
%TRAJECTORYPLOT  Plot target and actual tracking trajectories for each ASV (multi-subplot).
%
%   trajectoryPlot(TarTra, SystemStates, N, EnvironStates, plotParas)
%   visualizes the reference (target) and actual trajectories for each vessel,
%   automatically arranging up to 4 subplots per figure (more ASVs, new figures),
%   along with static/dynamic obstacles and ASV icons.
%
%   Inputs:
%     TarTra        - Cell array, TarTra{j} is [N×3] target trajectory for ASV j (x, y, psi)
%     SystemStates  - Cell array, SystemStates{j}.realStates [N×3] for ASV j (x, y, psi)
%     N             - Integer, number of time steps to plot
%     EnvironStates - Struct, environment info (may include .staticObs and .manual_dynamic)
%     plotParas     - Struct, plotting parameters including:
%                       .colors: color settings for ASVs and obstacles
%                       .iconInterval: interval for ASV icons
%
%   Output: None (opens one or more figures with trajectory subplots)
%
%   Example:
%     trajectoryPlot(MemTarTraSave,SystemStates,N,EnvironStates,plotParas)
%
%   Author: Wenxiang Wu (with ChatGPT enhancement)
%   Date:   2025-06-20

colors = plotParas.colors;
iconInterval = plotParas.iconInterval;
ASVNum = length(SystemStates);
xi = cell(1, ASVNum);
xd = cell(1, ASVNum);

% Automatically compute axisLimit (ignoring dynamic obstacles)  
x_all = [];
y_all = [];
for j = 1:ASVNum
    xi{j} = SystemStates{j}.realStates;
    xd{j} = TarTra{j};
    x_all = [x_all; xi{j}(1:N,1); xd{j}(1:N,1)];
    y_all = [y_all; xi{j}(1:N,2); xd{j}(1:N,2)];
end
if isfield(EnvironStates, "staticObs")
    for k = 1:numel(EnvironStates.staticObs)
        obs = EnvironStates.staticObs{k};
        x_all = [x_all; obs.Pos(:,1)];
        y_all = [y_all; obs.Pos(:,2)];
    end
end
margin = 5;
axisLimit.xmin = min(y_all) - margin;
axisLimit.xmax = max(y_all) + margin;
axisLimit.ymin = min(x_all) - margin;
axisLimit.ymax = max(x_all) + margin;

for j = 1:ASVNum
    % === Create a new figure every 4 ASVs ===   
    if mod(j-1, 4) == 0
        figure;
    end
    subplot(2, 2, mod(j-1, 4)+1);
    hold on
    axis([axisLimit.xmin axisLimit.xmax axisLimit.ymin axisLimit.ymax]);
    axis equal;
    title(['ASV ', num2str(j), ' Tracking Trajectory']);
    xlabel('y (m)', 'FontName', 'Times New Roman');
    ylabel('x (m)', 'FontName', 'Times New Roman');
    set(gca, 'FontName', 'Times New Roman');

    % ==== 1. Static Obstacles ====  
    if isfield(EnvironStates, "staticObs")
        for k = 1:numel(EnvironStates.staticObs)
            obs = EnvironStates.staticObs{k};
            fill(obs.Pos(:,2), obs.Pos(:,1), colors.manObs, 'FaceAlpha', .5, 'EdgeColor', 'k');
        end
    end

    % ==== 2. Dynamic Obstacles (for display only) ====  
    if isfield(EnvironStates, "manual_dynamic") && isfield(EnvironStates.manual_dynamic, "TS") && ...
       isfield(colors, "dynObs") && ~isempty(colors.dynObs)
        dynASVs = EnvironStates.manual_dynamic.TS;
        for k = 1:numel(dynASVs)
            dyn_traj = dynASVs{k}.Pos;
            plot(dyn_traj(:,2), dyn_traj(:,1), ':', 'Color', colors.dynObs(k,:), 'LineWidth', 1.5);
            psi = dynASVs{k}.Hdg(end);
            ASVDisplay3([psi 0 0], dyn_traj(end,2), dyn_traj(end,1), 0, 0.7, colors.dynObs(k,:));
        end
    end

    % ==== 3. Target and Actual Trajectories ====  
    p{1} = plot(xd{j}(1:N,2), xd{j}(1:N,1), 'Color', [0 0 0], 'LineWidth', 2); % 目标轨迹
    p{2} = plot(xi{j}(1:N,2), xi{j}(1:N,1), '--', 'Color', colors.ASV{j}, 'LineWidth', 2); % 实际轨迹

    % ==== 4. ASV Icons ==== 
    for i = 1:floor(N/iconInterval)
        ASVDisplay3([xi{j}(1+(i-1)*iconInterval,3),0,0], xi{j}(1+(i-1)*iconInterval,2), xi{j}(1+(i-1)*iconInterval,1), [], [], colors.ASV{j});
    end
    ASVDisplay3([xi{j}(N,3),0,0], xi{j}(N,2), xi{j}(N,1), [], [], colors.ASV{j});

    legend([p{:}], {
        ['ASV ' num2str(j) ' Target Trajectory'],
        ['ASV ' num2str(j) ' Actual Trajectory']
    });
end
end
