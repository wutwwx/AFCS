% SFCT Toolbox Path Update Script
%
% This script searches for the 'SFCT' (Ship Formation Control Toolbox)
% directory, removes any outdated SFCT paths from MATLAB, and then adds
% the latest SFCT path (including all subfolders) to the environment.
% Run this script whenever you update or relocate your SFCT toolbox.
%
% Author: Wenxiang/ChatGPT
% Date:   2025-06-19

% Locate directories named 'SFCT' using the MATLAB 'what' function
sfctInfo = what('SFCT');

% Check if any 'SFCT' directories were found
if isempty(sfctInfo)
    error(['SFCT toolbox directory not found. Please add the SFCT path ' ...
        'with subfolders from the menu or use addpath(genpath(...)).']);
elseif length(sfctInfo) > 1
    % Handle multiple SFCT directories found: use the first and notify the user
    warning('%d SFCT directories found. Using the first one found at: %s', ...
        length(sfctInfo), char(sfctInfo(1).path));
    basePath = char(sfctInfo(1).path);
else
    % Only one SFCT directory found: proceed
    basePath = char(sfctInfo.path);
    fprintf('SFCT directory found at: %s\n', basePath);
end

% Remove all old SFCT paths to avoid path conflicts
warning('off', 'MATLAB:rmpath:DirNotFound');
rmpath(genpath(basePath));
warning('on', 'MATLAB:rmpath:DirNotFound');

% Add the SFCT directory and all its subfolders to MATLAB's path
addpath(genpath(basePath));

% Save the updated MATLAB path for future sessions
savepath;

fprintf('MATLAB path for SFCT toolbox has been updated and saved successfully.\n');
