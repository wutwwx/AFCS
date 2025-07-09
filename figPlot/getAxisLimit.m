function axisLimit= getAxisLimit(SystemStates, TarTra, EnvironStates)
%GETAXISLIMIT   Compute axis limits for plotting ship formation and obstacle scenarios.
%
%   axisLimit = getAxisLimit(SystemStates, TarTra, EnvironStates) automatically
%   determines the x/y axis bounds for 2D plotting by scanning all relevant ship 
%   trajectories, targets, and obstacle points. Adds a small margin for visibility.
%
%   Inputs:
%     SystemStates   - Cell array, each cell contains a struct with .realStates (N×3 or N×6)
%     TarTra         - Array or cell array of reference trajectories, e.g. [N×3] or {ship1Tra, ...}
%     EnvironStates  - Struct, can include:
%                         .manual_dynamic.TS: cell, each contains .Pos (N×2) for dynamic obstacles
%                         .staticObs: cell, each contains .Pos (polygon points) for static obs
%
%   Output:
%     axisLimit      - Struct with fields: xmin, xmax, ymin, ymax, suitable for axis(...)
%
%   Example:
%     axisLimit = getAxisLimit(SystemStates, MemTarTraSave, EnvironStates);
%     axis([axisLimit.xmin axisLimit.xmax axisLimit.ymin axisLimit.ymax]);
%
%   Author: Wenxiang Wu (with ChatGPT enhancement)
%   Date:   2025-06-20

x_all = []; y_all = [];

% 1. Ship target trajectories  
if nargin > 1 && ~isempty(TarTra)
    if iscell(TarTra)
        for jj = 1:length(TarTra)
            x_all = [x_all; TarTra{jj}(:,2)];
            y_all = [y_all; TarTra{jj}(:,1)];
        end
    else %    Supports TarTra as an array  
        x_all = [x_all; TarTra(:,2)];
        y_all = [y_all; TarTra(:,1)];
    end
end

% 2. Actual ship trajectories  
for jj = 1:length(SystemStates)
    xi = SystemStates{jj}.realStates;
    x_all = [x_all; xi(:,2)];
    y_all = [y_all; xi(:,1)];
end

% 3. Dynamic obstacle trajectories  
if nargin > 2 && isfield(EnvironStates, "manual_dynamic") && isfield(EnvironStates.manual_dynamic, "TS")
    dynShips = EnvironStates.manual_dynamic.TS;
    for k = 1:numel(dynShips)
        x_all = [x_all; dynShips{k}.Pos(:,2)];
        y_all = [y_all; dynShips{k}.Pos(:,1)];
    end
end

% 4. Static obstacle boundaries  
if  isfield(EnvironStates, "staticObs")
    for k = 1:numel(EnvironStates.staticObs)
        obs = EnvironStates.staticObs{k};
        x_all = [x_all; obs.Pos(:,2)];
        y_all = [y_all; obs.Pos(:,1)];
    end
end

% 5. Add margins  
margin = 1; % Customizable
axisLimit.xmin = min(x_all) - margin;
axisLimit.xmax = max(x_all) + margin;
axisLimit.ymin = min(y_all) - margin;
axisLimit.ymax = max(y_all) + margin;
end
