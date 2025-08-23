function plotParas=plotParaSetting(EnvironStates,SystemStates, MemTarTraSave)
% plotParaSetting   Set standardized plotting parameters for multi-ASV/formation visualization.
%
%   plotParas = plotParaSetting(EnvironStates, SystemStates, MemTarTraSave)
%   returns a structure containing color schemes and drawing preferences for
%   plotting ASV trajectories, formation connection lines, obstacles, and
%   other scene elements in formation simulation.
%
%   Inputs:
%     EnvironStates    - Environment structure containing obstacle fields (static/dynamic)
%     SystemStates     - Cell array of system states for all ASVs (used for axis calculation)
%     MemTarTraSave    - Target trajectory history (used for axis calculation)
%
%   Outputs:
%     plotParas        - Struct with standardized plotting parameters:
%         .colors.ASVTra   - Target trajectory color (RGB)
%         .colors.connect   - Formation connection line color (RGB)
%         .colors.ASV      - Cell of RGB colors for each ASV (default: up to 4)
%         .colors.dynObs    - Colors for dynamic obstacles (uses lines() colormap if present)
%         .colors.manObs    - Static obstacle color (gray)
%         .axisLimit        - Plot axis ranges (struct)
%         .iconInterval     - Marker interval for ASV icons
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

plotParas.colors.ASVTra=[0,0,0];
plotParas.colors.connect=[0.4,0.4,0.4];
plotParas.colors.ASV{1}=[204, 0, 0]/255;
plotParas.colors.ASV{2}=[255, 192, 0]/255;
plotParas.colors.ASV{3}=[112,48, 160]/255;
plotParas.colors.ASV{4}=[0, 115, 52]/255;
if isfield(EnvironStates, "manual_dynamic") && isfield(EnvironStates.manual_dynamic, "TS")
    dynASVs = EnvironStates.manual_dynamic.TS;
    plotParas.colors.dynObs = lines(numel(dynASVs));
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
