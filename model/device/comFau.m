function shipsComFaults=comSet(shipNum,nowState,t)
%COMSET   Generate inter-ship communication link/fault status for multi-agent systems.
%
%   shipsComFaults = comSet(shipNum, nowState, t) returns a cell array
%   describing the communication state (connected/disconnected) and delay
%   for each pair of ships at current time t, based on their positions and 
%   a maximum communication range threshold.
%
%   Inputs:
%     shipNum      : Number of ships (agents)
%     nowState     : [shipNum × state_dim] matrix, each row is the state of one ship
%                    (first two columns are assumed [x, y] positions)
%     t            : Current simulation time (s)
%
%   Outputs:
%     shipsComFaults: Cell array (1×shipNum), each cell is a struct with fields:
%                      - .states : [1×shipNum] logical vector (1=linked, 0=lost)
%                      - .delays : [1×shipNum] communication delay (currently zeros)
%
%   Description:
%     - Two ships are considered linked if their Euclidean distance is less than the maximum range (default: 100,000 m).
%     - .states(j,i) = 1 if ship j can communicate with ship i, otherwise 0.
%     - Only Boolean (0/1) values are allowed for link state; otherwise, an error is thrown.
%     - Can be extended to model random, time-varying, or probabilistic communication faults.
%
%   Author: Wenxiang Wu
%   Date:   2025-03-30
%
%   Example usage:
%     shipsComFaults = comSet(shipNum, nowState, t);
ranges=100000*ones(shipNum,shipNum); 
for j=1:shipNum
    shipsComFaults{j}.delays=zeros(1,shipNum); 
    for i=1:shipNum
        distance(j,i)=sqrt((nowState(j,1)-nowState(i,1))^2+(nowState(j,2)-nowState(i,2))^2);
        if distance(j,i)>ranges(j,i)
            shipsComFaults{j}.states(1,i)=0;
        else
            shipsComFaults{j}.states(1,i)=1;
        end
    end
end

isAllLogical =all(shipsComFaults{j}.states == 0 | shipsComFaults{j}.states == 1);
if isAllLogical==false
    error('Communication states can only be set as Boolean variables');
end
