function ASVsComFaults=comSet(ASVNum,nowState,t)
%COMSET   Generate inter-ASV communication link/fault status for multi-agent systems.
%
%   ASVsComFaults = comSet(ASVNum, nowState, t) returns a cell array
%   describing the communication state (connected/disconnected) and delay
%   for each pair of ASVs at current time t, based on their positions and 
%   a maximum communication range threshold.
%
%   Inputs:
%     ASVNum      : Number of ASVs (agents)
%     nowState     : [ASVNum × state_dim] matrix, each row is the state of one ASV
%                    (first two columns are assumed [x, y] positions)
%     t            : Current simulation time (s)
%
%   Outputs:
%     ASVsComFaults: Cell array (1×ASVNum), each cell is a struct with fields:
%                      - .states : [1×ASVNum] logical vector (1=linked, 0=lost)
%                      - .delays : [1×ASVNum] communication delay (currently zeros)
%
%   Description:
%     - Two ASVs are considered linked if their Euclidean distance is less than the maximum range (default: 100,000 m).
%     - .states(j,i) = 1 if ASV j can communicate with ASV i, otherwise 0.
%     - Only Boolean (0/1) values are allowed for link state; otherwise, an error is thrown.
%     - Can be extended to model random, time-varying, or probabilistic communication faults.
%
%   Author: Wenxiang Wu
%   Date:   2025-03-30
%
%   Example usage:
%     ASVsComFaults = comSet(ASVNum, nowState, t);
ranges=100000*ones(ASVNum,ASVNum); 
for j=1:ASVNum
    ASVsComFaults{j}.delays=zeros(1,ASVNum); 
    for i=1:ASVNum
        distance(j,i)=sqrt((nowState(j,1)-nowState(i,1))^2+(nowState(j,2)-nowState(i,2))^2);
        if distance(j,i)>ranges(j,i)
            ASVsComFaults{j}.states(1,i)=0;
        else
            ASVsComFaults{j}.states(1,i)=1;
        end
    end
end

isAllLogical =all(ASVsComFaults{j}.states == 0 | ASVsComFaults{j}.states == 1);
if isAllLogical==false
    error('Communication states can only be set as Boolean variables');
end
