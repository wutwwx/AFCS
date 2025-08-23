function ASVsSenFaults=senFau(ASVNum,Nx,nowState,t)
%SENFAU   Generate sensor fault simulation signals for multiple ASVs.
%
%   ASVsSenFaults = senFau(ASVNum, Nx, nowState, t) produces a cell array 
%   describing the health state and fault-induced output errors for each 
%   ASV's sensors at the current time t.
%
%   Inputs:
%     ASVNum       : Number of ASVs
%     Nx            : [1 × ASVNum] vector, state dimension (number of sensors) per ASV
%     nowState      : [ASVNum × max(Nx)] matrix, current true state for each ASV
%     t             : Current simulation time (s)
%
%   Outputs:
%     ASVsSenFaults: Cell array (1×ASVNum), each cell is a struct with fields:
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
%     ASVsSenFaults = senFau(ASVNum, Nx, nowState, t);
ASVsSenFaults=cell(1,ASVNum);
s=0.3;
S=cell(1,size(nowState,1));
bias=0.1;
for j=1:ASVNum
    S{j}=s*eye(Nx(1,j));
    ASVsSenFaults{j}.states=ones(1,Nx(1,j)); 
    ASVsSenFaults{j}.faults=((S{j}-eye(Nx(1,j)))*nowState(j,:)'+bias*ones(size(nowState(j,:),2),1))'; 
end

isAllLogical =all(ASVsSenFaults{j}.states == 0 | ASVsSenFaults{j}.states == 1);
if isAllLogical==false
    error('Sensor states can only be set as Boolean variables');
end

end
