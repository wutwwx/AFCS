function plotASVPairwiseDistances(SystemStates, N, Ts)
%PLOTASVPAIRWISEDISTANCES  Plot pairwise distances between ASVs over time.
%
%   plotASVPairwiseDistances(SystemStates, N, Ts)
%   computes and plots the time-varying distance between every unique pair of ASVs
%   over the simulation. For each ASV pair, the minimum distance over all time steps
%   is printed to the command window.
%
%   Inputs:
%     SystemStates   - Cell array, SystemStates{j}.realStates [N×3], state trajectory for each ASV
%     N              - Integer, number of simulation steps
%     Ts             - Scalar, simulation time step (seconds)
%
%   Outputs:
%     (None. Displays a figure with pairwise distance curves and prints minimum values.)
%
%   Usage Example:
%     plotASVPairwiseDistances(SystemStates, N, Ts)
%
%   Author: Wenxiang Wu (with ChatGPT enhancement)
%   Date:   2025-06-21


ASVNum = length(SystemStates);
xy = zeros(N, 2, ASVNum);
for j = 1:ASVNum
    xy(:,:,j) = SystemStates{j}.realStates(1:N, 1:2); % N × 2
end

timeVec = (0:N-1)*Ts;
styles = {'-', '--', ':', '-.'};
colmap = lines(nchoosek(ASVNum,2));
curveIdx = 0;
minDistMat = zeros(ASVNum, ASVNum); % Store the minimum distance for each ASV pair   

figure; hold on; box on;
set(gca, 'FontName', 'Times New Roman');
legendItems = {};

for i = 1:ASVNum-1
    for j = i+1:ASVNum
        curveIdx = curveIdx + 1;
        d = sqrt(sum((xy(:,:,i) - xy(:,:,j)).^2, 2));
        plot(timeVec, d, ...
            'LineStyle', styles{mod(curveIdx-1,length(styles))+1}, ...
            'Color', colmap(curveIdx,:), 'LineWidth', 1.7);
        legendItems{end+1} = sprintf('ASV%d - ASV%d', i, j);
        minDistMat(i, j) = min(d);
    end
end

xlabel('Time (s)', 'FontName', 'Times New Roman');
ylabel('Distance (m)', 'FontName', 'Times New Roman');
title('Pairwise ASV Distances', 'FontName', 'Times New Roman');
legend(legendItems, 'Location', 'best');
grid on;
hold off;

% --- Print Minimum Distances --- 
fprintf('Minimum distance between each ASV pair during simulation:\n');
for i = 1:ASVNum-1
    for j = i+1:ASVNum
        fprintf('  ASV %d & ASV %d: %.3f m\n', i, j, minDistMat(i, j));
    end
end
end
