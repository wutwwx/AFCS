function displayASV(ASVName, dynamics, Number)
%DISPLAYASV   Display information and an image for a given ASV in a dedicated figure.
%
%   displayASV(ASVName, dynamics, figNumber)
%   Opens (or creates) figure window 'figNumber' and displays formatted information
%   (length, beam, mass) along with a ASV image, all within a fixed layout. Ensures
%   no interference with other figures by using explicit axes parenting.
%
%   Inputs:
%     ASVName   - String ('Yunfan1', 'Cybership2', ...), ASV model name.
%     dynamics   - Struct with ASV parameters (.L, .B, .mass, ...).
%     figNumber  - Integer, figure window number to use.
%
%   Example:
%     displayASV('Cybership2', ASVDynamics, 2)
%
%   Author: Haihong Wang (with ChatGPT enhancement)
%   Date:   2025-02-19

    % Create or activate the designated figure, then clear it
    hFig = figure(Number);
    clf(hFig);
    set(hFig, 'NumberTitle','off', 'Name',sprintf('ASV %d: %s', Number, ASVName));
    
    % Title and image file selection
    switch ASVName
        case "Yunfan1"
            Heading = sprintf('%-30s\n%-30s\n',  'Yunfan1');
            imageFile='yunfan1.jpg';
        case "Cybership2"
            Heading = sprintf('%-30s\n%-30s\n',  'Cybership2');
            imageFile='cyberShip2.png';
        otherwise
            error("No image for " + ASVName);
    end

    % ASV data formatting
    ASVData= {'Length (m)', dynamics.L, 'Beam (m)', dynamics.B, 'Mass (kg)', dynamics.mass};
    numEntries = length(ASVData)/2;
    formatSpec = repmat('%-20s : %-10.2f\n', 1, numEntries);
    ASV_text = sprintf(formatSpec, ASVData{:});
    
    % Create axes and plot all content on hFig ONLY
    axTitle = axes('Parent', hFig, 'Position', [0.1 0.8 0.8 0.1]);
    text(0, 1, Heading, ...
        'FontSize', 25, 'FontWeight', 'bold', 'FontName', 'Courier', ...
        'HorizontalAlignment', 'left', 'Parent', axTitle);
    axis(axTitle, 'off');
    
    axData = axes('Parent', hFig, 'Position', [0.1 0.53 0.8 0.3]);
    text(0, 1, ASV_text, ...
        'FontSize', 16, 'FontName', 'Courier', ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', ...
        'Parent', axData);
    axis(axData, 'off');
    
    % >>> This is KEY: create an axes in the target figure, and imshow to that axes <<<
    filePath = which(imageFile);
    axImg = axes('Parent', hFig, 'Position', [0.1 0.05 0.8 0.4]);
    imshow(imread(filePath), 'Parent', axImg);
end
