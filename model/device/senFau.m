function shipsSenFaults=senFau(shipNum,Nx,nowState,t)
%SENFAU   Generate sensor fault simulation signals for multiple ships.
%
%   shipsSenFaults = senFau(shipNum, Nx, nowState, t) produces a cell array 
%   describing the health state and fault-induced output errors for each 
%   ship's sensors at the current time t.
%
%   Inputs:
%     shipNum       : Number of ships
%     Nx            : [1 × shipNum] vector, state dimension (number of sensors) per ship
%     nowState      : [shipNum × max(Nx)] matrix, current true state for each ship
%     t             : Current simulation time (s)
%
%   Outputs:
%     shipsSenFaults: Cell array (1×shipNum), each cell is a struct with fields:
%                      - .states : [1×Nx(j)] logical vector (1=healthy, 0=faulty) for each sensor
%                      - .faults : [1×Nx(j)] additive sensor fault bias/error vector
%
%   Description:
%     - Simulates sensor faults as an additive bias on the measured state.
%     - Default: all sensors healthy (`.states = 1`), with a fixed bias error injected.
%     - Sensor states must be Boolean (0/1); otherwise, an error is raised.
%     - Customization possible for random, intermittent, or drifting sensor faults.
%
%   Author: Wenxiang Wu
%   Date:   2025-03-30
%
%   Example usage:
%     shipsSenFaults = senFau(shipNum, Nx, nowState, t);
shipsSenFaults=cell(1,shipNum);
s=0.3;
S=cell(1,size(nowState,1));
bias=0.1;
for j=1:shipNum
    S{j}=s*eye(Nx(1,j));
    shipsSenFaults{j}.states=ones(1,Nx(1,j)); 
    shipsSenFaults{j}.faults=((S{j}-eye(Nx(1,j)))*nowState(j,:)'+bias*ones(size(nowState(j,:),2),1))'; 
end

isAllLogical =all(shipsSenFaults{j}.states == 0 | shipsSenFaults{j}.states == 1);
if isAllLogical==false
    error('Sensor states can only be set as Boolean variables');
end

end