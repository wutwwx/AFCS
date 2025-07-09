function Prediction=mPCController(conInputs,commandLast,xd,ship,Ts)% xd includes the target point at the current time step 
%MPCONTROLLER  Nonlinear Model Predictive Controller for Ship Tracking.
%
%   Prediction = mPCController(conInputs, commandLast, xd, ship, Ts)
%   computes the optimal control command for a ship based on nonlinear MPC.
%   The controller tracks a given reference trajectory and handles constraints
%   on input, speed, and disturbance effects (e.g., current, unmodeled forces).
%
%   Inputs:
%     conInputs    - Struct with fields:
%                      .statesNow : [1×nx] current ship state
%                      .UnDis     : [1×3] (optional) unknown disturbances (default zeros)
%                      .current   : [1×3] (optional) current velocity/disturbance (default zeros)
%     commandLast  - [1×Nu] previous control command for warm start (can be empty)
%     xd           -  reference trajectory (first row is current ref)
%     ship         - Struct with fields:
%                      .controller : MPC parameter struct (fields: Q, R, G, Np, Nc, Nu)
%                      .dynamics   : Ship dynamic model struct (fields: MinInputs, MaxInputs, etc.)
%     Ts           - Scalar, sampling time (in seconds)
%
%   Outputs:
%     Prediction   - Struct with fields:
%                      .commands         : [1×Nu] optimal control at current step
%                      .commandsSequence : [Np×Nu] planned control sequence
%                      .preStates        : [Np×nx] predicted future states (after each step)
%                      .preNextStates    : [1×nx] predicted state at t+1
%
%   Usage Example:
%     Prediction = mPCController(conInputs, commandLast, xd, ship, Ts);
%
%   Author: Wenxiang Wu
%   Date:   2025-05-11

xi=conInputs.statesNow;
if ~isfield(conInputs,'UnDis')
    disturbance.force=zeros(1,3);
else
    disturbance.force=conInputs.UnDis;
end

if ~isfield(conInputs,'current')
    disturbance.current=zeros(1,3);
else
    disturbance.current=conInputs.current;
end

para=ship.controller;
dynamics=ship.dynamics;
if size(xd,1)==1
    xd=[zeros(1,size(xd,2));xd];
end
% If the controller prediction horizon is greater than the number of future target trajectory points, reduce the current prediction horizon;  
% If the control horizon is greater than the current prediction horizon, set the control horizon equal to the current prediction horizon  
Np=size(xd,1)-1;
if Np>para.Np
    Np=para.Np;
end
Nc=para.Nc;
if Nc>Np
    Nc=Np;
end    


if isempty(commandLast)
    commandLast=zeros(1,para.Nu);
end
u0=repmat(commandLast, 1, Nc);

minInputs = dynamics.MinInputs;
maxInputs = dynamics.MaxInputs;
lb = repmat(minInputs, Nc, 1)';
ub = repmat(maxInputs, Nc, 1)';
max_speed=[1.5,0.5,0.4];
min_speed=[-1.5,-0.5,-0.4];


u_optimal=fmincon(@(u)costfun(u,xi,xd(:,1:3),Np,Nc,para,dynamics,disturbance,max_speed,min_speed),u0,[],[] ,[],[],lb,ub, []);  
U = reshape(u_optimal(1,:), para.Nu, []).'; 
idx_1 = min((1:Np)', Nc); 
Prediction.commandsSequence = U(idx_1, :);
States(1,:)=xi;
for i=1:Np
    zz=PreDynamicsModel(dynamics,States(i,:)',Prediction.commandsSequence(i,:)',disturbance);
    States(i+1,:)=States(i,:)+Ts*zz(:,1)';
end
Prediction.preStates=States(2:Np+1,:);
Prediction.preNextStates=States(2,:);
Prediction.commands=Prediction.commandsSequence(1,:);
function  J=costfun(u,xi_pred,xd,Np,Nc,para,dynamics,disturbance,max_speed,min_speed)
    J=0;
    for j = 1:Np
        idx_2=min(j,Nc);
        tau=u((idx_2-1)*para.Nu+1:idx_2*para.Nu);
        diff = xi_pred(1,3) - xd(1,3);
        xi_pred(1,3) = xi_pred(1,3) - 2*pi * (diff > pi) + 2*pi * (diff < -pi);
        z=PreDynamicsModel(dynamics,xi_pred',tau',disturbance);
        xi_pred(1,:) = xi_pred(1,:)+Ts*z(:,1)';
        over_max = max(0, abs(xi_pred(1,4:6)) - max_speed);
        below_min = max(0, min_speed - abs(xi_pred(1,4:6)));
        over_limit=max(abs(over_max),abs(below_min));
        J=J+(xi_pred(1:3)-xd(j+1,:))*para.Q*(xi_pred(1:3)-xd(j+1,:))'+tau*para.R*tau'+over_limit*para.G*over_limit';   
    end
end

end