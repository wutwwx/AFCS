% AFCS Path Update Script
%
% This script searches for the 'AFCS' (Autonomous Surface Vehicle Formation Control Simulator)
% directory, removes any outdated AFCS paths from MATLAB, and then adds
% the latest AFCS path (including all subfolders) to the environment.
% Run this script whenever you update or relocate your AFCS.
%
% Author: Wenxiang/ChatGPT
% Date:   2025-06-19

% Locate directories named 'AFCS' using the MATLAB 'what' function
AFCSInfo = what('AFCS');

% Check if any 'AFCS' directories were found
if isempty(AFCSInfo)
    error(['AFCS directory not found. Please add the AFCS path ' ...
        'with subfolders from the menu or use addpath(genpath(...)).']);
elseif length(AFCSInfo) > 1
    % Handle multiple AFCS directories found: use the first and notify the user
    warning('%d AFCS directories found. Using the first one found at: %s', ...
        length(AFCSInfo), char(AFCSInfo(1).path));
    basePath = char(AFCSInfo(1).path);
else
    % Only one AFCS directory found: proceed
    basePath = char(AFCSInfo.path);
    fprintf('AFCS directory found at: %s\n', basePath);
end

% Remove all old AFCS paths to avoid path conflicts
warning('off', 'MATLAB:rmpath:DirNotFound');
rmpath(genpath(basePath));
warning('on', 'MATLAB:rmpath:DirNotFound');

% Add the AFCS directory and all its subfolders to MATLAB's path
addpath(genpath(basePath));

% Save the updated MATLAB path for future sessions
savepath;

fprintf('MATLAB path for AFCS has been updated and saved successfully.\n');
