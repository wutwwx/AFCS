function ASVs =  generateFormation(varargin)
%GENERATEFORMATION   Create a fleet of ASV structures, assign dynamics, observer, and controller.
%
%   ASVs = generateFormation(ASV_num, ASVNames, Observers, Controllers)
%
%   Inputs:
%     ASV_num    - Integer. Number of ASVs in the formation.
%     ASVNames   - Cell array or string array of ASV names ('Cybership2', 'Yunfan1', etc.)
%                   (Can be a single name for homogeneous formation)
%     Observers   - Cell array of observer parameter structs (empty for none).
%     Controllers - Cell array of controller parameter structs (must match ASV_num).
%
%   Output:
%     ASVs       - 1×ASV_num cell array. Each entry contains a struct with:
%                   .dynamics   -- ASV model parameter struct
%                   .observer   -- Observer parameter struct (optional)
%                   .controller -- Controller parameter struct
%
%   Example:
%     ASVs = generateFormation(3, {'Cybership2','Yunfan1','Cybership2'}, Observers, Controllers);
%
%   The function checks for size consistency and parameter validity.
%   Each ASV's info is displayed with displayASV.
%
%   Author: Wenxiang Wu
%   Date:   2025-02-09

    ASV_num = varargin{1};
    ASVNames = varargin{2};
    Observers= varargin{3};
    Controllers= varargin{4};
    if length(Observers)>ASV_num   
        error('The number of observers is more than the number of ASVs');
    end    
    if length(Controllers)>ASV_num   
        error('The number of controllers is more than the number of ASVs');
    end        
    
    if length(ASVNames) ~= ASV_num && length(ASVNames)~=1
        error('The number of ASVs is not equal to the number of ASV names');
    end        
    for j = 1 : ASV_num
        if length(ASVNames) ~= 1
            ASVs{j}.dynamics = createASV(ASVNames{j});% Import the ASV dynamic parameters
            displayASV(ASVNames{j}, ASVs{j}.dynamics, j); % Display ASV information
        else
                ASVs{j}.dynamics = createASV(ASVNames{1});% Import the ASV dynamic parameters
                displayASV(ASVNames{1}, ASVs{1}.dynamics, j); % Display ASV information
        end      
    end
    if ~isempty(Observers)
        for j = 1 : ASV_num
            if ~isempty(Observers{j})
                if Observers{j}.Nx<ASVs{j}.dynamics.Nx
                    error(['The ' num2str(j) 'th ASV observer sets less observe states than model states']);
                end 
                if Observers{j}.Nu>ASVs{j}.dynamics.Nu
                    error(['The ' num2str(j) 'th ASV observer sets more observe states than model inputs']);
                end
                ASVs{j}.observer=Observers{j}; % Import the obsrever parameters
            end
            if Controllers{j}.Nu>ASVs{j}.dynamics.Nu
                error(['The ' num2str(j) 'th ASV controller sets more control inputs than model inputs']);
            end            
            ASVs{j}.controller=Controllers{j}; % Import the controller parameters
        end
    else
        for j = 1 : ASV_num
            if Controllers{j}.Nu>ASVs{j}.dynamics.Nu
                error(['The ' num2str(j) 'th ASV controller sets more control inputs than model inputs']);
            end  
            ASVs{j}.controller=Controllers{j}; % Import the controller parameters
        end
    end   
    pause(0.01);
end




