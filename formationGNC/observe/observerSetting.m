function Observers = observerSetting(ShiNum)
%OBSERVERSETTING  Initialize observer parameters for each ship.
%
%   Observers = observerSetting(ShiNum)
%   Returns a cell array of observer parameter structs for each ship in the fleet.
%   This function currently supports the nonlinear disturbance observer described in:
%     Yang et al., "A Trajectory Tracking Robust Controller of Surface Vessels With Disturbance Uncertainties,"
%     IEEE Transactions on Industrial Electronics, 2019.
%
%   Inputs:
%     ShiNum    - Number of ships (integer)
%
%   Outputs:
%     Observers - 1×ShiNum cell array, each cell is a struct with fields:
%                   .Name      (string), observer algorithm name
%                   .Nx        (int),    state dimension (for compatibility)
%                   .Nu        (int),    input dimension (for compatibility)
%                   .K_omega   (matrix), observer gain
%
%   Usage Example:
%     Observers = observerSetting(4);
%
%   Author: Wenxiang Wu
%   Date:   2025-03-04

if ShiNum~=0
    Observers=cell(1,ShiNum);
    %% ship{1}
          Observers{1}.Name        = "DisObsYang"; % Only used for Fossen model
          Observers{1}.Nx          = 9;
          Observers{1}.Nu          = 3;
          Observers{1}.K_omega     =[
                                    1.7 0 0
                                    0 1.7 0
                                    0 0 1.7];
    %% ship{2}

    
    
    %% ship{3}



    %% ship{4} 
    
else
    Observers =[];
end

end
