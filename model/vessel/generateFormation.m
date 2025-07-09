function Ships =  generateFormation(varargin)
%GENERATEFORMATION   Create a fleet of ship structures, assign dynamics, observer, and controller.
%
%   Ships = generateFormation(Ship_num, ShipNames, Observers, Controllers)
%
%   Inputs:
%     Ship_num    - Integer. Number of ships in the formation.
%     ShipNames   - Cell array or string array of ship names ('Cybership2', 'Yunfan1', etc.)
%                   (Can be a single name for homogeneous formation)
%     Observers   - Cell array of observer parameter structs (empty for none).
%     Controllers - Cell array of controller parameter structs (must match Ship_num).
%
%   Output:
%     Ships       - 1×Ship_num cell array. Each entry contains a struct with:
%                   .dynamics   -- Ship model parameter struct
%                   .observer   -- Observer parameter struct (optional)
%                   .controller -- Controller parameter struct
%
%   Example:
%     Ships = generateFormation(3, {'Cybership2','Yunfan1','Cybership2'}, Observers, Controllers);
%
%   The function checks for size consistency and parameter validity.
%   Each ship's info is displayed with displayShip.
%
%   Author: Wenxiang Wu
%   Date:   2025-02-09

    Ship_num = varargin{1};
    ShipNames = varargin{2};
    Observers= varargin{3};
    Controllers= varargin{4};
    if length(Observers)>Ship_num   
        error('The number of observers is more than the number of ships');
    end    
    if length(Controllers)>Ship_num   
        error('The number of controllers is more than the number of ships');
    end        
    
    if length(ShipNames) ~= Ship_num && length(ShipNames)~=1
        error('The number of ships is not equal to the number of ship names');
    end        
    for j = 1 : Ship_num
        if length(ShipNames) ~= 1
            Ships{j}.dynamics = createShip(ShipNames{j});% Import the ship dynamic parameters
            displayShip(ShipNames{j}, Ships{j}.dynamics, j); % Display ship information
        else
                Ships{j}.dynamics = createShip(ShipNames{1});% Import the ship dynamic parameters
                displayShip(ShipNames{1}, Ships{1}.dynamics, j); % Display ship information
        end      
    end
    if ~isempty(Observers)
        for j = 1 : Ship_num
            if ~isempty(Observers{j})
                if Observers{j}.Nx<Ships{j}.dynamics.Nx
                    error(['The ' num2str(j) 'th ship observer sets less observe states than model states']);
                end 
                if Observers{j}.Nu>Ships{j}.dynamics.Nu
                    error(['The ' num2str(j) 'th ship observer sets more observe states than model inputs']);
                end
                Ships{j}.observer=Observers{j}; % Import the obsrever parameters
            end
            if Controllers{j}.Nu>Ships{j}.dynamics.Nu
                error(['The ' num2str(j) 'th ship controller sets more control inputs than model inputs']);
            end            
            Ships{j}.controller=Controllers{j}; % Import the controller parameters
        end
    else
        for j = 1 : Ship_num
            if Controllers{j}.Nu>Ships{j}.dynamics.Nu
                error(['The ' num2str(j) 'th ship controller sets more control inputs than model inputs']);
            end  
            Ships{j}.controller=Controllers{j}; % Import the controller parameters
        end
    end   
    pause(0.01);
end




