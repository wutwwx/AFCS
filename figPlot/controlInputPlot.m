function controlInputPlot(SystemStates, N, colors)
%CONTROLINPUTPLOT  Plot control inputs of multiple ships over simulation steps.
%
%   controlInputPlot(SystemStates, N, colors) visualizes the time-series control
%   commands (inputs) applied to each ship/ASV/USV over the simulation duration.
%
%   Inputs:
%     SystemStates : cell array, each cell contains states and control inputs for a ship.
%                    SystemStates{j}.commands is an [N × nU] matrix of control variables.
%                    nU = number of control variables (e.g. 3 for Fossen, 2 for MMG).
%     N            : scalar, number of simulation time steps to plot.
%     colors       : struct, must include field .ship, a cell array of RGB row vectors
%                    or color codes for each ship (e.g., colors.ship{j} = [r g b]).
%
%   Description:
%     - Supports arbitrary number of ships and any number of control variables.
%     - Variable naming is automatic for common marine models (Fossen, MMG, etc).
%     - Line styles are cycled to distinguish different ships.
%     - Each subplot shows the evolution of a single control variable.
%
%   Example:
%       % Assuming SystemStates is already generated in your simulation...
%       controlInputPlot(SystemStates,N,plotParas.colors);
%
%   Notes:
%     - This function is compatible with Fossen-type and MMG-type models.
%     - For more than 3 control variables, labels will default to 'u_1', 'u_2', ...
%
%   Author: Wenxiang Wu (with ChatGPT enhancement)
%   Date:   2025-06-20

ShipNum = length(SystemStates);
for j= 1:ShipNum
    U{j}=SystemStates{j}.commands;
end

ShipNum = length(U);
nU = size(U{1},2); % number of the control variables
% the names of the control variables, which can be customized according to actual needs.
if nU==3
    uNames = {'Surge Force (\tau_x)', 'Sway Force (\tau_y)', 'Yaw Moment (\tau_\psi)'};
elseif nU==2
    uNames = {'Propeller', 'Rudder'};
else
    uNames = arrayfun(@(k) ['u_', num2str(k)], 1:nU, 'UniformOutput', false);
end
linestyles = {'-', '--', ':', '-.'}; % separation between ships

figure('Name', 'Control Inputs', 'Color', 'w');
for k = 1:nU
    subplot(nU,1,k); hold on; box on; grid on;
    for j = 1:ShipNum
        plot(1:N, U{j}(1:N, k), ...
            'Color', colors.ship{j}, ...
            'LineStyle', linestyles{mod(j-1,length(linestyles))+1}, ...
            'LineWidth', 2, ...
            'DisplayName', ['Ship ', num2str(j)]);
    end
    ylabel(uNames{k}, 'FontName', 'Times New Roman');
    set(gca, 'FontName', 'Times New Roman');
    if k==1
        title('Control Inputs vs. Time');
    end
    if k==nU
        xlabel('Time Step', 'FontName', 'Times New Roman');
    end
    legend show
end
end
