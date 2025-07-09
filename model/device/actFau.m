function shipsActFaults=actFau(shipNum,Nu,nowCommands,t)
%ACTFAU   Generate actuator fault simulation signals for multi-ship control.
%
%   shipsActFaults = actFau(shipNum, Nu, nowCommands, t) constructs cell arrays 
%   describing the actuator status and fault effects for a fleet of ships at time t.
%   Each ship's actuator state is defined as logical (1=normal, 0=faulty), and the 
%   corresponding fault signal is added to the commanded input vector.
%
%   Inputs:
%     shipNum       : Number of ships in the formation/scenario
%     Nu            : [1 × shipNum] vector, number of control inputs per ship
%     nowCommands   : [shipNum × ?] matrix, current commanded control input for each ship
%     t             : Current simulation time (s)
%
%   Outputs:
%     shipsActFaults: Cell array (1×shipNum), each cell is a struct with fields:
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
%     shipsActFaults = actFau(shipNum, Nu, nowCommands, t);
shipsActFaults=cell(1,shipNum);
s=0.3;
S=cell(1,size(nowCommands,1));
bias=0.1;
for j=1:shipNum
    shipsActFaults{j}.states=ones(1,Nu(1,j)); 
    shipsActFaults{j}.faults=((S{j}-eye(Nx(1,j)))*nowCommands(j,:)'+bias*ones(size(nowCommands(j,:),2),1))';
end
isAllLogical =all(shipsActFaults{j}.states == 0 | shipsActFaults{j}.states == 1);
if isAllLogical==false
    error('Actuator states can only be set as Boolean variables');
end