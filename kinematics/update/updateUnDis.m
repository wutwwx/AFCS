function systemNext=updateUnDis(N,ships,system,conSystem,env,Ts,i) % 
%UPDATEUNDIS  System state update for scenarios with unknown disturbance injection.
%
%   systemNext = updateUnDis(N, ships, system, conSystem, env, Ts, i)
%   Advances system and disturbance states by one time step, applying the unknown disturbance profile.
%
%   Inputs:
%     N         - Integer, total simulation steps
%     ships     - Cell array, ship model structures (with dynamics, controller, observer, etc.)
%     system    - Cell array, current system states (each with fields .realStates, .commands, etc.)
%     conSystem - Cell array, control system outputs (controller/observer updates and commands)
%     env       - Struct, environment states (not directly used in this function)
%     Ts        - Scalar, simulation sampling time [s]
%     i         - Integer, current simulation step (1-based)
%
%   Outputs:
%     systemNext - Cell array, updated system states (all fields updated to next step)
%
%   Usage Example:
%     systemNext = updateUnDis(N, ships, system, conSystem, env, Ts, i)
%
%   Author: Wenxiang Wu
%   Date:   2025-07-01

shipNum=length(system);
systemNext=system;
realStatesNow=[];
commandsNow=[];

for j=1:shipNum
    systemNext{j}.commands(i,:)=conSystem{j}.controller.commands(i,:);
    if isfield(ships{j},'observer')
        systemNext{j}.observer=conSystem{j}.observer;
    end
    if isfield(ships{j},'controller')
        systemNext{j}.controller=conSystem{j}.controller;
    end     
    realStatesNow(end+1, :)=system{j}.realStates(i,:);
    commandsNow(end+1, :)=system{j}.commands(i,:);
end
UnDis=unknownDisturbance(shipNum,realStatesNow,Ts*(i));
if i<N   
    for j=1:shipNum
        disturbance{j}.current=[0 0 0];
        disturbance{j}.force=system{j}.UnDis(i,:);
        states=system{j}.realStates(i,:);
        zz=dynamicsModel(ships{j}.dynamics,states',systemNext{j}.commands(i,:)',disturbance{j});
        nextStates=states+Ts*zz';
        systemNext{j}.realStates(i+1,:)=nextStates;      
        systemNext{j}.UnDis(i+1,:)=UnDis{j};
    end
end
end