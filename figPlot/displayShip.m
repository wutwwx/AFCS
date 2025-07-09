function displayShip(shipName, dynamics, figNumber)
%DISPLAYSHIP   Display information and an image for a given ship in a dedicated figure.
%
%   displayShip(shipName, dynamics, figNumber)
%   Opens (or creates) figure window 'figNumber' and displays formatted information
%   (length, beam, mass) along with a ship image, all within a fixed layout. Ensures
%   no interference with other figures by using explicit axes parenting.
%
%   Inputs:
%     shipName   - String ('Yunfan1', 'Cybership2', ...), ship model name.
%     dynamics   - Struct with ship parameters (.L, .B, .mass, ...).
%     figNumber  - Integer, figure window number to use.
%
%   Example:
%     displayShip('Cybership2', shipDynamics, 2)
%
%   Author: Haihong Wang (with ChatGPT enhancement)
%   Date:   2025-02-19

    % Create or activate the designated figure, then clear it
    hFig = figure(figNumber);
    clf(hFig);
    set(hFig, 'NumberTitle','off', 'Name',['Ship Info: ' char(shipName)]);
    
    % Title and image file selection
    switch shipName
        case "Yunfan1"
            Heading = sprintf('%-30s\n%-30s\n',  'Renjiantianjin');
            imageFile='yunfan1.jpg';
        case "Cybership2"
            Heading = sprintf('%-30s\n%-30s\n',  'Cybership2');
            imageFile='cyberShip2.png';
        otherwise
            error("No image for " + shipName);
    end

    % Ship data formatting
    shipData= {'Length (m)', dynamics.L, 'Beam (m)', dynamics.B, 'Mass (kg)', dynamics.mass};
    numEntries = length(shipData)/2;
    formatSpec = repmat('%-20s : %-10.2f\n', 1, numEntries);
    Ship_text = sprintf(formatSpec, shipData{:});
    
    % Create axes and plot all content on hFig ONLY
    axTitle = axes('Parent', hFig, 'Position', [0.1 0.8 0.8 0.1]);
    text(0, 1, Heading, ...
        'FontSize', 25, 'FontWeight', 'bold', 'FontName', 'Courier', ...
        'HorizontalAlignment', 'left', 'Parent', axTitle);
    axis(axTitle, 'off');
    
    axData = axes('Parent', hFig, 'Position', [0.1 0.53 0.8 0.3]);
    text(0, 1, Ship_text, ...
        'FontSize', 16, 'FontName', 'Courier', ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', ...
        'Parent', axData);
    axis(axData, 'off');
    
    % >>> This is KEY: create an axes in the target figure, and imshow to that axes <<<
    filePath = which(imageFile);
    axImg = axes('Parent', hFig, 'Position', [0.1 0.05 0.8 0.4]);
    imshow(imread(filePath), 'Parent', axImg);
end
