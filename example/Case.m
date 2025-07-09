clear all; 
close all;
warning off;
N=200; % Set simulation steps
Ts=1; % Set simulation interval
ShipNum = 4; % Set Number of formation members
ShipNames = "Cybership2";  % Select the ship models" ["Cybership2","Yunfan1"]
Observers=observerSetting(ShipNum); % Set observers
Controllers=controllerSetting(ShipNum); % Set controllers
Ships=generateFormation(ShipNum,ShipNames,Observers,Controllers); % Configure ships
ObstacleTypes=["manual_static","manual_dynamic"];% Set obstacle types  ["manual_static","rand_static","manual_dynamic"]
MapFile=[]; % Set navigation map
EnvironStates=generateEnvironment(MapFile,ObstacleTypes, N, Ts, 1);  % Configure environments
SystemDisturbances = ["unknownDisturbance"];% Set system disturbances, ["winds", "waves", "currents", "measureNoise", "sensorFault", "actuatorFault", "communicationFault","unknownDisturbance"]
SystemStates=systemInitial(N,Ships,EnvironStates,SystemDisturbances); % Initial system states
dataUpdate=systemUpdate(SystemDisturbances); % Initial data update function
Task=taskManualSet();  % Set formation objectives 
ForTra=ForPathToTra(Task,N,Ts);  % Generate target formation trajectory
ControlStrategy=strategySetting("VirtualStructure",Task,ShipNum); % Set control strategy
[ForTarTra,MemTarTra]=traGeneration(Task,ForTra,ControlStrategy,SystemStates,[],Ts,1); % Generate target trajectories for each member
ControlStates= controlInitial(N,Ships,MemTarTra,SystemStates,EnvironStates,Ts); % Calculate control commands and relative control states
SystemStates=dataUpdate(N,Ships,SystemStates,ControlStates,EnvironStates,1,Ts); % Update system states
ForTarTraSave(1,:) =ForTarTra(1,:); % Save formation target trajectory
for j=1:ShipNum
    MemTarTraSave{j}(1,:) =MemTarTra{j}(1,:); % Save all member target trajectories
end
for i=2:N
    [ForTarTra,MemTarTra] = traGeneration(Task,ForTra,ControlStrategy,SystemStates,ControlStates,Ts,i); % Generate target trajectories for each member
    ControlStates = formationControl(Ships,MemTarTra,SystemStates,ControlStates,EnvironStates,Ts,i); % Calculate control commands and relative control states
    SystemStates=dataUpdate(N,Ships,SystemStates,ControlStates,EnvironStates,Ts,i); % Update system states
    ForTarTraSave(i,:) =ForTarTra(1,:); % Save formation target trajectory
    for j=1:ShipNum
        MemTarTraSave{j}(i,:) =MemTarTra{j}(1,:); % Save all member target trajectories
    end
end  
plotParas=plotParaSetting(EnvironStates,SystemStates, MemTarTraSave); % Set plot parameters
formationPlot(ForTarTraSave,SystemStates,N, EnvironStates,plotParas)  % Static display formation trajectory
formationAnimate(SystemStates, N, EnvironStates,plotParas)            % Dynamic display formation trajectory
trajectoryPlot(MemTarTraSave,SystemStates,N,EnvironStates,plotParas)  % Static display all ship tracking results
trajectoryAnimate(MemTarTraSave,SystemStates,N,EnvironStates,plotParas) % Dynamic display all ship tracking results
trackingErrorPlot(MemTarTraSave,SystemStates,N,plotParas) % Display tracking errors
controlInputPlot(SystemStates,N,plotParas.colors) % Display control inputs
observePlot(SystemStates, N, plotParas.colors) % Display observer outputs
plotShipAllObsDistances(SystemStates, N, Ts, EnvironStates) % Plot distances between each ship and all obstacles
plotShipPairwiseDistances(SystemStates, N, Ts) % Plot pairwise distances between ships over time



