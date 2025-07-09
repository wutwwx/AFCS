function plotParas=plotParaSetting(EnvironStates,SystemStates, MemTarTraSave)
% plotParaSetting   Set standardized plotting parameters for multi-ship/formation visualization.
%
%   plotParas = plotParaSetting(EnvironStates, SystemStates, MemTarTraSave)
%   returns a structure containing color schemes and drawing preferences for
%   plotting ship trajectories, formation connection lines, obstacles, and
%   other scene elements in formation simulation.
%
%   Inputs:
%     EnvironStates    - Environment structure containing obstacle fields (static/dynamic)
%     SystemStates     - Cell array of system states for all ships (used for axis calculation)
%     MemTarTraSave    - Target trajectory history (used for axis calculation)
%
%   Outputs:
%     plotParas        - Struct with standardized plotting parameters:
%         .colors.shipTra   - Target trajectory color (RGB)
%         .colors.connect   - Formation connection line color (RGB)
%         .colors.ship      - Cell of RGB colors for each ship (default: up to 4)
%         .colors.dynObs    - Colors for dynamic obstacles (uses lines() colormap if present)
%         .colors.manObs    - Static obstacle color (gray)
%         .axisLimit        - Plot axis ranges (struct)
%         .iconInterval     - Marker interval for ship icons
%         .animateInterval  - Animation step interval
%         .animateSpeed     - Animation frame delay (seconds)
%
%   Description:
%     - Used to ensure consistent colors and plotting behavior across all visualization functions.
%     - Automatically adapts obstacle colors to current environment contents.
%     - Recommended for use with formationPlot, formationAnimate, trajectoryPlot, etc.
%
%   Example:
%       plotParas = plotParaSetting(EnvironStates,SystemStates, MemTarTraSave);
%
%   Author: Wenxiang Wu (with ChatGPT enhancement)
%   Date:   2025-06-20

plotParas.colors.shipTra=[0,0,0];
plotParas.colors.connect=[0.4,0.4,0.4];
plotParas.colors.ship{1}=[204, 0, 0]/255;
plotParas.colors.ship{2}=[255, 192, 0]/255;
plotParas.colors.ship{3}=[112,48, 160]/255;
plotParas.colors.ship{4}=[0, 115, 52]/255;
if isfield(EnvironStates, "manual_dynamic") && isfield(EnvironStates.manual_dynamic, "TS")
    dynShips = EnvironStates.manual_dynamic.TS;
    plotParas.colors.dynObs = lines(numel(dynShips));
else
    plotParas.colors.dynObs = [];
end
if isfield(EnvironStates, "manual_static") && isfield(EnvironStates, "rand_static") 
    plotParas.colors.manObs = [0.4 0.4 0.4];
else
    plotParas.colors.manObs = [];
end
plotParas.axisLimit=getAxisLimit(SystemStates, MemTarTraSave, EnvironStates);
plotParas.iconInterval=30;
plotParas.animateInterval=1;
plotParas.animateSpeed=0.02;
end