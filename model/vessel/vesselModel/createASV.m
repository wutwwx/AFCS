function dynamics = createASV(varargin)
%CREATEASV   Create ASV dynamics parameters struct based on the ASV model name.
%
%   dynamics = createASV(ASVName)
%   Returns a struct containing all necessary physical and hydrodynamic parameters for
%   the specified ASV model. The actual data struct is loaded via model-specific functions.
%
%   Inputs:
%     ASVName   - String, ASV model name ('Cybership2' or 'Yunfan1')
%
%   Output:
%     dynamics   - Struct, model parameters for the requested ASV
%
%   Example:
%     dynamics = createASV('Cybership2');
%
%   Author: Wenxiang Wu
%   Date:   2025-02-25

    if varargin{1}=="Cybership2"
        dynamics=cybership2;
    elseif varargin{1}=="Yunfan1"
        dynamics=yunfan1;
    else
        error("The ASV model of " + varargin + " does not exist");
    end   
end

