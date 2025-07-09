function Prediction=mPCControllerCA(conInputs,controlStates,EnvironStates,commandLast,xd,ship,Ts,st) %xd includes the target point at the current time step  
%MPCONTROLLERCA  Nonlinear MPC with Collision Avoidance for Multi-Ship/USV System.
%
%   Prediction = mPCControllerCA(conInputs, controlStates, EnvironStates, ...
%                                commandLast, xd, ship, Ts, st)
%   computes the optimal control action using nonlinear MPC with collision
%   avoidance constraints, handling static obstacles, dynamic obstacles,
%   and internal coordination (e.g., formation inter-ship avoidance).
%
%   Inputs:
%     conInputs     - Struct, current state info for this ship
%                      .statesNow : [1×nx] current state
%                      .UnDis     : [1×3] unknown disturbance (optional)
%                      .current   : [1×3] current disturbance (optional)
%     controlStates - Cell array, other ships' predicted trajectories (for formation avoidance)
%     EnvironStates - Struct, environmental states
%                       .staticObs         : cell, static obstacle info
%                       .manual_dynamic.TS : cell, dynamic obstacle trajectories
%                       .manual_dynamic.R  : [1×Ndyn], radii for dynamic obstacles
%     commandLast   - [1×Nu], previous control input (for warm start, can be empty)
%     xd            -  reference trajectory (first row is current ref)
%     ship          - Struct, contains:
%                        .controller : MPC tuning parameters (Q, R, G, Np, Nc, Nu, etc.)
%                        .dynamics   : Ship dynamic parameters/model
%     Ts            - Scalar, sampling time [s]
%     st            - Integer, current global time step (for indexing into dynamic obs)
%
%   Outputs:
%     Prediction    - Struct, fields:
%                        .commands          : [1×Nu], optimal command at current step
%                        .commandsSequence  : [Np×Nu], optimal sequence over horizon
%                        .preStates         : [Np×nx], predicted future states
%                        .preNextStates     : [1×nx], predicted state at t+1
%
%   Usage Example:
%     Prediction = mPCControllerCA(conInputs, controlStates, EnvironStates, ...
%                                  commandLast, xd, ship, Ts, st)
%
%   Reference:
%     [1] Wen G, Lam J, Fu J, Wang S.
%         "Distributed MPC-based robust collision avoidance formation navigation of constrained multiple USVs."
%          IEEE Transactions on Intelligent Vehicles. 2023 Sep 14;9(1):1804-16.
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
if isfield(EnvironStates, "manual_dynamic") && isfield(EnvironStates.manual_dynamic, "TS") && ~isempty(EnvironStates.manual_dynamic.TS)
    TS_Num = length(EnvironStates.manual_dynamic.TS);
    dynTS   = EnvironStates.manual_dynamic.TS;
    dynR    = EnvironStates.manual_dynamic.R;
else
    TS_Num = 0;
    dynTS = {};    
    dynR  = [];    
end

% If the controller's prediction horizon is greater than the number of future target trajectory points, reduce the current prediction horizon;  
% If the control horizon is greater than the current prediction horizon, set the control horizon equal to the current prediction horizon  
Np=size(xd,1)-1;
if Np>para.Np
    Np=para.Np;
end
Nc=para.Nc;
if Nc>Np
    Nc=Np;
end  

for h=1:TS_Num
    n1 = size(dynTS{h}.Pos, 1);
    n2 = st+Np;
    if n1 < n2
        last_row = dynTS{h}.Pos(end, :);
        add_rows = repmat(last_row, n2 - n1, 1);
        dynTS{h}.Pos = [dynTS{h}.Pos; add_rows];
    end
end
% Revert to EnvironStates.manual_dynamic.TS, if needed 
if TS_Num > 0
    EnvironStates.manual_dynamic.TS = dynTS;
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




nonlcon = @(u) myNonlcon(u, xi, xd(:,1:3), Np, Nc, para, dynamics, disturbance, controlStates,EnvironStates,st,max_speed,min_speed);
u_optimal=fmincon(@(u)costfun(u,xi,xd(:,1:3),Np,Nc,para,dynamics,disturbance,max_speed,min_speed),u0,[],[] ,[],[],lb,ub,nonlcon); 
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
            idx=min(j,Nc);
            tau=u((idx-1)*para.Nu+1:idx*para.Nu);
			diff = xi_pred(1,3) - xd(1,3);
			xi_pred(1,3) = xi_pred(1,3) - 2*pi * (diff > pi) + 2*pi * (diff < -pi);
            z=PreDynamicsModel(dynamics,xi_pred',tau',disturbance);
            xi_pred=xi_pred+Ts*z';
            over_max = max(0, abs(xi_pred(1,4:6)) - max_speed);
            below_min = max(0, min_speed - abs(xi_pred(1,4:6)));
            over_limit=max(abs(over_max),abs(below_min));
            J=J+(xi_pred(1:3)-xd(j+1,:))*para.Q*(xi_pred(1:3)-xd(j+1,:))'+tau*para.R*tau'+over_limit*para.G*over_limit'; 
        end
    end

    function [c, ceq] = myNonlcon(u,xi_pred,xd,Np,Nc,para,dynamics,disturbance,controlStates,EnvironStates,st,max_speed,min_speed)
        dynamicObs = [];
        conNum_2 = numel(dynamicObs);
        if isfield(EnvironStates, "manual_dynamic") && isfield(EnvironStates.manual_dynamic, "TS") && ~isempty(EnvironStates.manual_dynamic.TS)
            dynamicObs = EnvironStates.manual_dynamic;
            conNum_2 = numel(dynamicObs.TS);
        end
        staticObs = [];
        if isfield(EnvironStates, "staticObs") && ~isempty(EnvironStates.staticObs)
            staticObs = EnvironStates.staticObs;
        end
        conNum_3 = numel(staticObs);
        conNum_1=length(controlStates);
        conNum=conNum_1+conNum_2+conNum_3;
        c=zeros((conNum+6)*Np,1);
        for j = 1:Np
            idx_2=min(j,Nc);
            tau=u((idx_2-1)*para.Nu+1:idx_2*para.Nu);
			diff = xi_pred(1,3) - xd(1,3);
			xi_pred(1,3) = xi_pred(1,3) - 2*pi * (diff > pi) + 2*pi * (diff < -pi);
            z = PreDynamicsModel(dynamics,xi_pred',tau',disturbance);
            xi_pred(1,:) = xi_pred(1,:)+Ts*z(:,1)';
            for k=1:conNum_1         
                if size(controlStates{k}.controller.preStates,1)<j
                    c((j-1)*conNum+k,1)=-1;
                else
                    c((j-1)*conNum+k,1)=1.5*dynamics.L-norm([xi_pred(1,1)-controlStates{k}.controller.preStates(j,1),xi_pred(1,2)-controlStates{k}.controller.preStates(j,2)]);
                end
            end
            for k=1:conNum_2
                c((j-1)*conNum+conNum_1+k,1)=dynamics.L+dynamicObs.R(1,k)-norm([xi_pred(1,1)-dynamicObs.TS{k}.Pos(st+j,1),xi_pred(1,2)-dynamicObs.TS{k}.Pos(st+j,2)]);
            end
            for k=1:conNum_3
                c((j-1)*conNum+conNum_1+conNum_2+k,1)=dynamics.L+staticObs{k}.Rad-norm([xi_pred(1,1)-staticObs{k}.center(1,1),xi_pred(1,2)-staticObs{k}.center(1,2)]);
            end           
            idx = conNum*Np + (j-1)*6;
            c(idx+1,1) = xi_pred(1,4) - max_speed(1,1);
            c(idx+2,1) = xi_pred(1,5) - max_speed(1,2);
            c(idx+3,1) = xi_pred(1,6) - max_speed(1,3);
            c(idx+4,1) = min_speed(1,1)-xi_pred(1,4);
            c(idx+5,1) = min_speed(1,2)-xi_pred(1,5);
            c(idx+6,1) = min_speed(1,3)-xi_pred(1,6);            
        end  
        ceq=[];
        

    end

end