function ASVsActFaults=actFau(ASVNum,Nu,nowCommands,t)
%ACTFAU   Generate actuator fault simulation signals for multi-ASV control.
%
%   ASVsActFaults = actFau(ASVNum, Nu, nowCommands, t) constructs cell arrays 
%   describing the actuator status and fault effects for a fleet of ASVs at time t.
%   Each ASV's actuator state is defined as logical (1=normal, 0=faulty), and the 
%   corresponding fault signal is added to the commanded input vector.
%
%   Inputs:
%     ASVNum       : Number of ASVs in the formation/scenario
%     Nu            : [1 × ASVNum] vector, number of control inputs per ASV
%     nowCommands   : [ASVNum × ?] matrix, current commanded control input for each ASV
%     t             : Current simulation time (s)
%
%   Outputs:
%     ASVsActFaults: Cell array (1×ASVNum), each cell is a struct with fields:
%                      - .states : [1×Nu(j)] logical vector (1=healthy, 0=faulty)
%                      - .faults : [1×Nu(j)] fault effect added to actuator (bias, loss, etc.)
%
%   Notes:
%     - Actuator state variables must be Boolean (0 or 1), otherwise an error is thrown.
%     - This function can be customized for time-varying, random, or specific actuator fault scenarios.
%     - Default: all actuators healthy, with fixed bias injected as fault effect.
%
%   Author: Wenxiang Wu
%   Date:   2025-03-30
%   Example usage:
%     ASVsActFaults = actFau(ASVNum, Nu, nowCommands, t);
ASVsActFaults=cell(1,ASVNum);
s=0.3;
S=cell(1,size(nowCommands,1));
bias=0.1;
for j=1:ASVNum
    ASVsActFaults{j}.states=ones(1,Nu(1,j)); 
    ASVsActFaults{j}.faults=((S{j}-eye(Nx(1,j)))*nowCommands(j,:)'+bias*ones(size(nowCommands(j,:),2),1))';
end
isAllLogical =all(ASVsActFaults{j}.states == 0 | ASVsActFaults{j}.states == 1);
if isAllLogical==false
    error('Actuator states can only be set as Boolean variables');
end
