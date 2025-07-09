function  conSystem= controlInitial(N,ships,xd,systemStates,EnvironStates,Ts)
%CONTROLINITIAL   Initialize controller and observer for the first simulation step.
%
%   conSystem = controlInitial(N, ships, xd, systemStates, EnvironStates, Ts)
%   Initializes controller and observer structures for all ships at step 1,
%   including allocation of command arrays, observer state, and initial control commands.
%
%   Inputs:
%     N            - Integer, total number of simulation steps
%     ships        - Cell array, ship model structures (with .dynamics, .controller, .observer, etc.)
%     xd           - Cell array, target trajectory or setpoint for each ship at the initial step
%     systemStates - Cell array, current states of each ship (with .realStates, etc.)
%     EnvironStates- Struct, environment info (for collision-aware controllers, etc.)
%     Ts           - Scalar, sampling time [s]
%
%   Outputs:
%     conSystem    - Cell array, initialized controller/observer system for each ship
%
%   Usage Example:
%     conSystem = controlInitial(N, ships, xd, systemStates, EnvironStates, Ts)
%
%   Author: Wenxiang Wu
%   Date:   2025-06-18

ShipNum=length(ships);
%% Initial
for j=1:ShipNum
    if isfield(ships{j},'observer')           
        observer{j}=ships{j}.observer;       
    end
    controller{j}=ships{j}.controller;
    conSystem{j}.controller.commands=zeros(N,ships{j}.dynamics.Nu);
    Inputs{j}.statesNow=systemStates{j}.realStates(1,:);
end
%% observe and control for the first step
for j=1:ShipNum
    if isfield(ships{j},'observer')
        switch observer{j}.Name
            case "DisObsYang"
                conSystem{j}.observer.Belta=[0,0,0];
                obserOutputs{j}.UnDis=[0,0,0];
            otherwise 
                error('The observe algorithm is not defined');
        end  
        observerfields = fieldnames(obserOutputs{j});        
        for k = 1:length(observerfields)
            conSystem{j}.observer.(observerfields{k})(1,:) = obserOutputs{j}.(observerfields{k});
        end 
        Inputs{j}.UnDis=obserOutputs{j}.UnDis;
    end
end
controllerOutputs = cell(ShipNum,1);
parfor j=1:ShipNum
    switch controller{j}.Name
        case "MPC"
            controllerOutputs{j}=mPCController(Inputs{j},[],xd{j},ships{j},Ts);
        case "MPCCA"
            controllerOutputs{j}=mPCControllerCA(Inputs{j},[],EnvironStates,[],xd{j},ships{j},Ts,1);           
        otherwise 
            error('The control algorithm is not defined');
    end  
    if ~isfield(controllerOutputs{j},'commands')
        error('The cotroller commands are not ouput')
    end  
    controllerfields = fieldnames(controllerOutputs{j});
    for k = 1:length(controllerfields)
        if controllerfields{k}=="commands"||controllerfields{k}=="preNextStates"
            conSystem{j}.controller.(controllerfields{k})(1,:) = controllerOutputs{j}.(controllerfields{k});
        else
            conSystem{j}.controller.(controllerfields{k})= controllerOutputs{j}.(controllerfields{k});
        end
    end    
end

end
