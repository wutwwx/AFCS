clear all; 
close all;
warning off;
N=200; % Set simulation steps
Ts=1; % Set simulation interval
ASVNum = 4; % Set Number of formation members
ASVNames = "Cybership2";  % Select the ASV models" ["Cybership2","Yunfan1"]
Observers=observerSetting(ASVNum); % Set observers
Controllers=controllerSetting(ASVNum); % Set controllers
ASVs=generateFormation(ASVNum,ASVNames,Observers,Controllers); % Configure ASVs
ObstacleTypes=["manual_static","manual_dynamic"];% Set obstacle types  ["manual_static","rand_static","manual_dynamic"]
MapFile=[]; % Set navigation map
EnvironStates=generateEnvironment(MapFile,ObstacleTypes, N, Ts, 1);  % Configure environments
SystemDisturbances = ["unknownDisturbance"];% Set system disturbances, ["winds", "waves", "currents", "measureNoise", "sensorFault", "actuatorFault", "communicationFault","unknownDisturbance"]
SystemStates=systemInitial(N,ASVs,EnvironStates,SystemDisturbances); % Initial system states
dataUpdate=systemUpdate(SystemDisturbances); % Initial data update function
Task=taskManualSet();  % Set formation objectives 
ForTra=ForPathToTra(Task,N,Ts);  % Generate target formation trajectory
ControlStrategy=strategySetting("VirtualStructure",Task,ASVNum); % Set control strategy
[ForTarTra,MemTarTra]=traGeneration(Task,ForTra,ControlStrategy,SystemStates,[],Ts,1); % Generate target trajectories for each member
ControlStates= controlInitial(N,ASVs,MemTarTra,SystemStates,EnvironStates,Ts); % Calculate control commands and relative control states
SystemStates=dataUpdate(N,ASVs,SystemStates,ControlStates,EnvironStates,1,Ts); % Update system states
ForTarTraSave(1,:) =ForTarTra(1,:); % Save formation target trajectory
for j=1:ASVNum
    MemTarTraSave{j}(1,:) =MemTarTra{j}(1,:); % Save all member target trajectories
end
for i=2:N
    [ForTarTra,MemTarTra] = traGeneration(Task,ForTra,ControlStrategy,SystemStates,ControlStates,Ts,i); % Generate target trajectories for each member
    ControlStates = formationControl(ASVs,MemTarTra,SystemStates,ControlStates,EnvironStates,Ts,i); % Calculate control commands and relative control states
    SystemStates=dataUpdate(N,ASVs,SystemStates,ControlStates,EnvironStates,Ts,i); % Update system states
    ForTarTraSave(i,:) =ForTarTra(1,:); % Save formation target trajectory
    for j=1:ASVNum
        MemTarTraSave{j}(i,:) =MemTarTra{j}(1,:); % Save all member target trajectories
    end
end  
plotParas=plotParaSetting(EnvironStates,SystemStates, MemTarTraSave); % Set plot parameters
formationPlot(ForTarTraSave,SystemStates,N, EnvironStates,plotParas)  % Static display formation trajectory
formationAnimate(SystemStates, N, EnvironStates,plotParas)            % Dynamic display formation trajectory
trajectoryPlot(MemTarTraSave,SystemStates,N,EnvironStates,plotParas)  % Static display all ASV tracking results
trajectoryAnimate(MemTarTraSave,SystemStates,N,EnvironStates,plotParas) % Dynamic display all ASV tracking results
trackingErrorPlot(MemTarTraSave,SystemStates,N,plotParas) % Display tracking errors
controlInputPlot(SystemStates,N,plotParas.colors) % Display control inputs
observePlot(SystemStates, N, plotParas.colors) % Display observer outputs
plotASVAllObsDistances(SystemStates, N, Ts, EnvironStates) % Plot distances between each ASV and all obstacles
plotASVPairwiseDistances(SystemStates, N, Ts) % Plot pairwise distances between ASVs over time



