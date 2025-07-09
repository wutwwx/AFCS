function Controllers = controllerSetting(ships)
%CONTROLLERSETTING  Generate controller parameter sets for all ships in the formation.
%
%   Controllers = controllerSetting(ships)
%   returns a cell array of controller configurations for each ship, including
%   MPC or collision-avoidance (MPCCA) variants, prediction and control horizon,
%   penalty matrices, tolerances, etc.
%
%   Inputs:
%     ships        - Cell array of ship model structures (not directly used, included for interface compatibility)
%
%   Outputs:
%     Controllers  - Cell array (1 × numShips). Each Controllers{j} is a struct containing parameters of controller
%
%   Example usage:
%     Controllers = controllerSetting(ships);
%
%   Author: Wenxiang Wu
%   Date:   2025-05-11

Controllers=cell(1,length(ships));
%% Cybership2
% ship{1}
    Controllers{1}.Name        = "MPCCA";   
    Controllers{1}.Fre         = 1;                                                    % control frequency
    Controllers{1}.Q           = diag([1,1,5]);                                        % state penalty
    Controllers{1}.R           = diag([0,0,0]);                                        % control penalty
    Controllers{1}.F           = diag([0,0]);                                          % move suppression penalty
    Controllers{1}.G           = diag([1000000 1000000 1000000]);                      % speed suppression penalty
    Controllers{1}.Np          = 10;                                                   % predicton horizon
    Controllers{1}.Nc          = 5;                                                    % control horizon
    Controllers{1}.Nx          = 6;                                                    % state number
    Controllers{1}.Nu          = 3;                                                    % input number
    Controllers{1}.Ny          = 2;                                                    % output number
    Controllers{1}.Ne          = 3;                                                    % disturbance vector
    Controllers{1}.atol_real   = 1e-6;                                                 % tolerances for real vehicle system
    Controllers{1}.rtol_real   = 1e-3;                            
    Controllers{1}.atol_sim    = 1e-6;                                                 % tolerances for vehcle model % no model mismatch
    Controllers{1}.rtol_sim    = 1e-3;

% ship{2}
    Controllers{2}.Name        = "MPCCA"; 
    Controllers{2}.Fre         = 1;                                                    % control frequency
    Controllers{2}.Q           = diag([1,1,5]);                                        % state penalty
    Controllers{2}.R           = diag([0,0,0]);                                        % control penalty
    Controllers{2}.F           = diag([0,0]);                                          % move suppression penalty
    Controllers{2}.G           = diag([1000000 1000000 1000000]);                      % speed suppression penalty
    Controllers{2}.Np          = 10;                                                   % predicton horizon
    Controllers{2}.Nc          = 5;                                                    % control horizon
    Controllers{2}.Nx          = 6;                                                    % state number
    Controllers{2}.Nu          = 3;                                                    % input number
    Controllers{2}.Ny          = 2;                                                    % output number
    Controllers{2}.Ne          = 3;                                                    % disturbance vector
    Controllers{2}.atol_real   = 1e-6;                                                 % tolerances for real vehicle system
    Controllers{2}.rtol_real   = 1e-3;                            
    Controllers{2}.atol_sim    = 1e-6;                                                 % tolerances for vehcle model % no model mismatch
    Controllers{2}.rtol_sim    = 1e-3;

% ship{3}
    Controllers{3}.Name        = "MPCCA";    
    Controllers{3}.Fre         = 1;                                                    % control frequency
    Controllers{3}.Q           = diag([1,1,5]);                                        % state penalty
    Controllers{3}.R           = diag([0,0,0]);                                        % control penalty
    Controllers{3}.F           = diag([0,0]);                                          % move suppression penalty
    Controllers{3}.G           = diag([1000000 1000000 1000000]);                      % speed suppression penalty
    Controllers{3}.Np          = 10;                                                   % predicton horizon
    Controllers{3}.Nc          = 5;                                                    % control horizon
    Controllers{3}.Nx          = 6;                                                    % state number
    Controllers{3}.Nu          = 3;                                                    % input number
    Controllers{3}.Ny          = 2;                                                    % output number
    Controllers{3}.Ne          = 3;                                                    % disturbance vector
    Controllers{3}.atol_real   = 1e-6;                                                 % tolerances for real vehicle system
    Controllers{3}.rtol_real   = 1e-3;                            
    Controllers{3}.atol_sim    = 1e-6;                                                 % tolerances for vehcle model % no model mismatch
    Controllers{3}.rtol_sim    = 1e-3;

% ship{4} 
    Controllers{4}.Name        = "MPCCA";
    Controllers{4}.Fre         = 1;                                                    % control frequency
    Controllers{4}.Q           = diag([1,1,5]);                                        % state penalty
    Controllers{4}.R           = diag([0,0,0]);                                        % control penalty
    Controllers{4}.F           = diag([0,0]);                                          % move suppression penalty
    Controllers{4}.G           = diag([1000000 1000000 1000000]);                      % speed suppression penalty
    Controllers{4}.Np          = 10;                                                   % predicton horizon
    Controllers{4}.Nc          = 5;                                                    % control horizon
    Controllers{4}.Nx          = 6;                                                    % state number
    Controllers{4}.Nu          = 3;                                                    % input number
    Controllers{4}.Ny          = 2;                                                    % output number
    Controllers{4}.Ne          = 3;                                                    % disturbance vector
    Controllers{4}.atol_real   = 1e-6;                                                 % tolerances for real vehicle system
    Controllers{4}.rtol_real   = 1e-3;                            
    Controllers{4}.atol_sim    = 1e-6;                                                 % tolerances for vehcle model % no model mismatch
    Controllers{4}.rtol_sim    = 1e-3;

% Yunfan1
%   % ship{1}
%     Controllers{1}.Name        = "MPC";   
%     Controllers{1}.Fre         = 1;                                                    % control frequency
%     Controllers{1}.Q           = diag([1,1,1]);                                        % state penalty
%     Controllers{1}.R           = diag([0,0]);                                        % control penalty
%     Controllers{1}.F           = diag([0,0]);                                          % move suppression penalty
%     Controllers{1}.G           = diag([1000000,1000000,1000000]);                      % speed suppression penalty
%     Controllers{1}.Np          = 10;                                                   % predicton horizon
%     Controllers{1}.Nc          = 5;                                                    % control horizon
%     Controllers{1}.Nx          = 6;                                                    % state number
%     Controllers{1}.Nu          = 2;                                                    % input number
%     Controllers{1}.Ny          = 2;                                                    % output number
%     Controllers{1}.Ne          = 3;                                                    % disturbance vector
%     Controllers{1}.atol_real   = 1e-6;                                                 % tolerances for real vehicle system
%     Controllers{1}.rtol_real   = 1e-3;                            
%     Controllers{1}.atol_sim    = 1e-6;                                                 % tolerances for vehcle model % no model mismatch
%     Controllers{1}.rtol_sim    = 1e-3;
% 
%   % ship{2}
%     Controllers{2}.Name        = "MPC"; 
%     Controllers{2}.Fre         = 1;                                                    % control frequency
%     Controllers{2}.Q           = diag([1,1,1]);                                        % state penalty
%     Controllers{2}.R           = diag([0,0]);                                        % control penalty
%     Controllers{2}.F           = diag([0,0]);                                          % move suppression penalty
%     Controllers{2}.G           = diag([1000000,1000000,1000000]);                      % speed suppression penalty
%     Controllers{2}.Np          = 10;                                                   % predicton horizon
%     Controllers{2}.Nc          = 5;                                                    % control horizon
%     Controllers{2}.Nx          = 6;                                                    % state number
%     Controllers{2}.Nu          = 2;                                                    % input number
%     Controllers{2}.Ny          = 2;                                                    % output number
%     Controllers{2}.Ne          = 3;                                                    % disturbance vector
%     Controllers{2}.atol_real   = 1e-6;                                                 % tolerances for real vehicle system
%     Controllers{2}.rtol_real   = 1e-3;                            
%     Controllers{2}.atol_sim    = 1e-6;                                                 % tolerances for vehcle model % no model mismatch
%     Controllers{2}.rtol_sim    = 1e-3;
% 
%   % ship{3}
%     Controllers{3}.Name        = "MPC";    
%     Controllers{3}.Fre         = 1;                                                    % control frequency
%     Controllers{3}.Q           = diag([1,1,1]);                                        % state penalty
%     Controllers{3}.R           = diag([0,0]);                                        % control penalty
%     Controllers{3}.F           = diag([0,0]);                                          % move suppression penalty
%     Controllers{3}.G           = diag([1000000,1000000,1000000]);                      % speed suppression penalty
%     Controllers{3}.Np          = 10;                                                   % predicton horizon
%     Controllers{3}.Nc          = 5;                                                    % control horizon
%     Controllers{3}.Nx          = 6;                                                    % state number
%     Controllers{3}.Nu          = 2;                                                    % input number
%     Controllers{3}.Ny          = 2;                                                    % output number
%     Controllers{3}.Ne          = 3;                                                    % disturbance vector
%     Controllers{3}.atol_real   = 1e-6;                                                 % tolerances for real vehicle system
%     Controllers{3}.rtol_real   = 1e-3;                            
%     Controllers{3}.atol_sim    = 1e-6;                                                 % tolerances for vehcle model % no model mismatch
%     Controllers{3}.rtol_sim    = 1e-3;
% 
%   % ship{4} 
%     Controllers{4}.Name        = "MPC";
%     Controllers{4}.Fre         = 1;                                                    % control frequency
%     Controllers{4}.Q           = diag([1,1,1]);                                        % state penalty
%     Controllers{4}.R           = diag([0,0]);                                        % control penalty
%     Controllers{4}.F           = diag([0,0]);                                          % move suppression penalty
%     Controllers{4}.G           = diag([1000000,1000000,1000000]);                      % speed suppression penalty
%     Controllers{4}.Np          = 10;                                                   % predicton horizon
%     Controllers{4}.Nc          = 5;                                                    % control horizon
%     Controllers{4}.Nx          = 6;                                                    % state number
%     Controllers{4}.Nu          = 2;                                                    % input number
%     Controllers{4}.Ny          = 2;                                                    % output number
%     Controllers{4}.Ne          = 3;                                                    % disturbance vector
%     Controllers{4}.atol_real   = 1e-6;                                                 % tolerances for real vehicle system
%     Controllers{4}.rtol_real   = 1e-3;                            
%     Controllers{4}.atol_sim    = 1e-6;                                                 % tolerances for vehcle model % no model mismatch
%     Controllers{4}.rtol_sim    = 1e-3;


end
