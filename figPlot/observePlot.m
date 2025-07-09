function observePlot(SystemStates, N, colors)
%OBSERVEPLOT  Plot true and estimated disturbances for each vessel.
%
%   observePlot(SystemStates, N, colors) visualizes the comparison between
%   the real (true) unknown disturbance and the observer-estimated disturbance
%   for each vessel, if an observer is present.
%
%   Inputs:
%     SystemStates - Cell array, each cell contains a struct with at least:
%                      .UnDis        : [N × D] matrix, true disturbance for D dimensions
%                      .observer     : struct (optional), with field:
%                         .UnDis     : [N × D] estimated disturbance
%     N            - Integer, number of time steps to plot
%     colors       - Struct, colors for each vessel: colors.ship{j}
%
%   Output:
%     None (plots figures for each ship with disturbance and estimation curves)
%
%   Example:
%     observePlot(SystemStates, N, plotParas.colors);
%
%   Author: Wenxiang Wu (with ChatGPT enhancement)
%   Date:   2025-06-20

ShipNum = length(SystemStates);
for j = 1:ShipNum
   % Check whether the 'observer' field is included   
    if isfield(SystemStates{j}, 'observer') && ~isempty(SystemStates{j}.observer)
        figure('Name', ['Ship ' num2str(j) ' Disturbance Observation'], 'NumberTitle', 'off');

        % Get the disturbance dimension  
        dim = size(SystemStates{j}.UnDis, 2);

        for i = 1:dim
            subplot(dim, 1, i);
            hold on;

            % Real disturbance and estimated disturbance curves  
            plot(SystemStates{j}.UnDis(1:N, i), 'k', 'LineWidth', 2);
            plot(SystemStates{j}.observer.UnDis(1:N, i), '--', 'Color', colors.ship{j}, 'LineWidth', 2);

            % Add legend and title 
            legend({'True Disturbance', 'Estimated Disturbance'});
            title(['Disturbance Dimension ' num2str(i)]);
            xlabel('Time Step');
            ylabel('Disturbance');

            grid on;
        end
    end
end
end
