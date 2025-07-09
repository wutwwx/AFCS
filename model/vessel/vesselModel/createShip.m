function dynamics = createShip(varargin)
%CREATESHIP   Create ship dynamics parameters struct based on the ship model name.
%
%   dynamics = createShip(shipName)
%   Returns a struct containing all necessary physical and hydrodynamic parameters for
%   the specified ship model. The actual data struct is loaded via model-specific functions.
%
%   Inputs:
%     shipName   - String, ship model name ('Cybership2' or 'Yunfan1')
%
%   Output:
%     dynamics   - Struct, model parameters for the requested ship
%
%   Example:
%     dynamics = createShip('Cybership2');
%
%   Author: Wenxiang Wu
%   Date:   2025-02-25

    if varargin{1}=="Cybership2"
        dynamics=cybership2;
    elseif varargin{1}=="Yunfan1"
        dynamics=yunfan1;
    else
        error("The ship model of " + varargin + " does not exist");
    end   
end

