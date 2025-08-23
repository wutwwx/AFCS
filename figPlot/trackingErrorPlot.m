function trackingErrorPlot(TarTra, SystemStates, N, plotParas)
%TRACKINGERRORPLOT  Plot trajectory tracking errors (distance & heading) for ASV formations.
%
%   trackingErrorPlot(TarTra, SystemStates, N, plotParas)
%   generates plots of Euclidean distance and heading (angle) tracking errors 
%   for each ASV in the formation, across all time steps.
%
%   Inputs:
%     TarTra        - Cell array, TarTra{j} [N×3], target trajectory for each ASV
%     SystemStates  - Cell array, SystemStates{j}.realStates [N×3], actual state for each ASV
%     N             - Integer, total number of time steps to plot
%     plotParas     - Struct, plotting parameters (colors, etc.)
%
%   Outputs:
%     (None, creates a figure window with tracking error plots)
%
%   Usage Example:
%     trackingErrorPlot(MemTarTraSave,SystemStates,N,plotParas)
%
%   Author: Wenxiang Wu (with ChatGPT enhancement)
%   Date:   2025-06-22

colors = plotParas.colors;
ASVNum = length(SystemStates);
linestyles = {'-', '--', ':', '-.', '-', '--', ':', '-.'}; % extensible

figure('Name', 'Tracking Errors', 'Color', 'w'); % one window


%% --- 1. Euclidean Distance Error ---    
subplot(2,1,1); hold on; box on; grid on;
title('Tracking Error (Euclidean Distance) vs. Time');
xlabel('Time Step', 'FontName', 'Times New Roman');
ylabel('Tracking Error (m)', 'FontName', 'Times New Roman');
set(gca, 'FontName', 'Times New Roman');
linesE = gobjects(ASVNum,1);
for j = 1:ASVNum
    xi = SystemStates{j}.realStates;
    xd = TarTra{j};
    err = sqrt((xi(1:N,1) - xd(1:N,1)).^2 + (xi(1:N,2) - xd(1:N,2)).^2);
    linesE(j) = plot(1:N, err, ...
        'Color', colors.ASV{j}, ...
        'LineStyle', linestyles{mod(j-1,length(linestyles))+1}, ...
        'LineWidth', 2, ...
        'DisplayName', ['ASV ', num2str(j)]);
end
legend(linesE, 'Location', 'best');

%% --- 2. Heading Error ---
subplot(2,1,2); hold on; box on; grid on;
title('Heading Error vs. Time');
xlabel('Time Step', 'FontName', 'Times New Roman');
ylabel('Heading Error (deg)', 'FontName', 'Times New Roman');
set(gca, 'FontName', 'Times New Roman');
linesH = gobjects(ASVNum,1);
for j = 1:ASVNum
    xi = SystemStates{j}.realStates;
    xd = TarTra{j};
    psi_e = rad2deg( wrapToPi( xi(1:N,3) - xd(1:N,3) ) );
    linesH(j) = plot(1:N, psi_e, ...
        'Color', colors.ASV{j}, ...
        'LineStyle', linestyles{mod(j-1,length(linestyles))+1}, ...
        'LineWidth', 2, ...
        'DisplayName', ['ASV ', num2str(j)]);
end
legend(linesH, 'Location', 'best');

end
