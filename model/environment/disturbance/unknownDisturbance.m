function disturbance=unknownDisturbance(shipNum,nowState,t)
%UNKNOWNDISTURBANCE   Generate unknown (external) disturbances for multi-ship scenarios.
%
%   disturbance = unknownDisturbance(shipNum, nowState, t) returns a cell array
%   of unknown additive disturbances for each ship at time t, to be used in 
%   simulation of model uncertainty or unmeasured environmental effects.
%
%   Inputs:
%     shipNum      : Number of ships
%     nowState     : [shipNum × state_dim] matrix, current state of each ship (not used by default)
%     t            : Current simulation time (s)
%
%   Outputs:
%     disturbance  : Cell array (1×shipNum), each cell is a [1×3] vector of
%                    additive disturbances [d_u, d_v, d_r] for surge, sway, and yaw
%
%   Description:
%     - By default, only ship 1 is assigned a time-varying unknown disturbance;
%       other ships' disturbances are zero.
%     - The disturbance consists of harmonic (sinusoidal) terms, simulating
%       unknown external effects (e.g., unmodeled environment, actuator bias).
%     - Function can be extended for more general/multiple-ship cases as needed.
%
%   Author: Wenxiang Wu
%   Date:   2025-02-25
%
%   Example usage:
%     disturbance = unknownDisturbance(shipNum, nowState, t);
for j=1:shipNum
    disturbance{j}=zeros(1,3);
end
disturbance{1}(1,1)=1.3/12+ 2/12*sin(0.02*t)+1.5/12*sin(0.1*t);
disturbance{1}(1,2)=-0.9/6+ 2/6*sin(0.02*t-pi/6)+1.5/6*sin(0.3*t);
disturbance{1}(1,3)=-1/12*sin(0.09*t + pi/3) -4/12*sin(0.01*t);