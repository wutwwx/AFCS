function Currents=currentsSet(shipNum,currents,realStatesNow,t)
%CURRENTSSET   Generate water current disturbances (earth and body frame) for multiple ships.
%
%   Currents = currentsSet(shipNum, currents, realStatesNow, t) produces the
%   current velocity vectors for each ship, both in the earth-fixed and
%   ship-fixed (body) frames, supporting time-varying or constant input.
%
%   Inputs:
%     shipNum        : Number of ships
%     currents       : [1×3] vector [u_east, v_north, r_current] (m/s, m/s, rad/s), earth-fixed current velocity.
%                      If empty, a time-varying test current is used.
%     realStatesNow  : [shipNum × state_dim] matrix of current ship states
%                      (state vector, with yaw angle as 3rd entry, e.g., [x, y, psi, ...])
%     t              : Current simulation time (s)
%
%   Outputs:
%     Currents       : Cell array (1×shipNum), each cell is a struct:
%                       - .earthSpeed : [1×3] current vector in earth-fixed frame (m/s, m/s, rad/s)
%                       - .bodySpeed  : [1×3] current vector in ship body-fixed frame
%
%   Description:
%     - Transforms earth-fixed current vector to ship-fixed frame using current heading.
%     - Supports both static (constant) and dynamic (time-varying) currents.
%     - Current transformation: bodySpeed = T * earthSpeed, where T is the standard rotation matrix.
%
%   Reference:
%     - Fossen, T. I. (2011). "Handbook of Marine Craft Hydrodynamics and Motion Control," Wiley.
%
%   Author: Wenxiang Wu
%   Date:   2025-02-25
%
%   Example usage:
%     Currents = currentsSet(shipNum, currents, realStatesNow, t);
if isempty(currents)
    earthSpeed(1,1) = 0.3*sin(t/100);
    earthSpeed(1,2) = 0;
    earthSpeed(1,3) = 0;
else
    earthSpeed(1,1) = currents(1,1);
    earthSpeed(1,2) = currents(1,2);
    earthSpeed(1,3) = currents(1,3);
end
Currents=cell(1,shipNum);%

for j=1:shipNum
    Currents{j}.earthSpeed(1,1) = earthSpeed(1,1);
    Currents{j}.earthSpeed(1,2) = earthSpeed(1,2);
    Currents{j}.earthSpeed(1,3) = earthSpeed(1,3);
    T{j}=[cos(realStatesNow(j,3)) sin(realStatesNow(j,3)) 0
         -sin(realStatesNow(j,3)) cos(realStatesNow(j,3)) 0 
                    0                      0              1
    ];
    Currents{j}.bodySpeed(1,:)=(T{j}*Currents{j}.earthSpeed')';
end


end