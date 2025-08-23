function conSystem=formationControl(ASVs,xd,systemStates,controlStates,EnvironStates,Ts,st)
%FORMATIONCONTROL  Multi-ASV formation control main routine (supports observer and MPC).
%
%   conSystem = formationControl(ASVs, xd, systemStates, controlStates, ...
%                                EnvironStates, Ts, i)
%   computes the control commands for all ASVs in the formation at time step i,
%   including observer updates and controller selection (MPC, MPC-CA), with support
%   for parallel acceleration. Results are written into the controller/observer fields
%   of each element in conSystem.
%
%   Inputs:
%     ASVs         - Cell array, each ASV struct with fields:
%                      .controller : controller configuration
%                      .observer   : (optional) observer config
%     xd            - Cell array, xd{j} is current reference (target) for each ASV
%     systemStates  - Cell array, .realStates is state history for each ASV
%     controlStates - Cell array, stores historical observer/controller output for each ASV
%     EnvironStates - Struct, environment (used for collision avoidance)
%     Ts            - Scalar, controller sample time [s]
%     st             - Integer, current time index
%
%   Output:
%     conSystem     - Cell array, updated controlStates with observer/controller fields
%
%   Usage Example:
%     conSystem = formationControl(ASVs, xd, systemStates, controlStates, EnvironStates, Ts, st)
%
%   Author: Wenxiang Wu
%   Date:   2025-07-11

conSystem=controlStates;
ASVNum=length(ASVs);
for j=1:ASVNum
    if isfield(ASVs{j},'observer')           
        observer{j}=ASVs{j}.observer;       
    end
    controller{j}=ASVs{j}.controller;
    Inputs{j}.statesNow=systemStates{j}.realStates(st,:);
    Inputs{j}.statesPast=systemStates{j}.realStates(st-1,:);
end
for j=1:ASVNum
    if isfield(ASVs{j},'observer')
        switch observer{j}.Name
            case "DisObsYang"
                obserOutputs{j}=disObsYang(Inputs{j},controlStates{j}.observer,controlStates{j}.controller.commands(st-1,:),ASVs{j},Ts);
            otherwise 
                error('The observe algorithm is not defined');
        end
        observerfields = fieldnames(obserOutputs{j});
        for k = 1:length(observerfields)
            if observerfields{k}=="UnDis"
                conSystem{j}.observer.(observerfields{k})(st,:) = obserOutputs{j}.(observerfields{k});
            else
                conSystem{j}.observer.(observerfields{k}) = obserOutputs{j}.(observerfields{k});
            end
        end     
        Inputs{j}.UnDis=obserOutputs{j}.UnDis;
    end
end
controllerOutputs = cell(ASVNum,1);


parfor j=1:ASVNum 
    commandLast=controlStates{j}.controller.commands(st-1,:);
    switch controller{j}.Name
        case "MPC"
            controllerOutputs{j}=mPCController(Inputs{j},commandLast,xd{j},ASVs{j},Ts);        
        case "MPCCA"  
            controlStatesinputs=controlStates;
            controlStatesinputs(j) = [];
            controllerOutputs{j}=mPCControllerCA(Inputs{j},controlStatesinputs,EnvironStates,commandLast,xd{j},ASVs{j},Ts,st);
        otherwise 
            error('The control algorithm is not defined');
    end       
    if ~isfield(controllerOutputs{j},'commands')
        error('The cotroller commands are not ouput')
    end  
    controllerfields = fieldnames(controllerOutputs{j});      
    for k = 1:length(controllerfields)
        if controllerfields{k}=="commands"||controllerfields{k}=="preNextStates"
            conSystem{j}.controller.(controllerfields{k})(st,:) = controllerOutputs{j}.(controllerfields{k});
        else
            conSystem{j}.controller.(controllerfields{k})= controllerOutputs{j}.(controllerfields{k});
        end
    end    
end


end
