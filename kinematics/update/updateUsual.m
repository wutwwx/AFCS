function systemNext=updateUsual(N,ASVs,system,conSystem,env,Ts,i) 
%UPDATEUSUAL  System state update for nominal (no-disturbance) scenarios.
%
%   systemNext = updateUsual(N, ASVs, system, conSystem, env, Ts, i)
%   Advances system states by one step under nominal (zero disturbance) conditions.
%
%   Inputs:
%     N         - Integer, total simulation steps
%     ASVs     - Cell array, ASV model structures (with .dynamics, .controller, etc.)
%     system    - Cell array, current system states (with fields .realStates, .commands, etc.)
%     conSystem - Cell array, controller/observer updates and computed commands
%     env       - Struct, environment states (not used here)
%     Ts        - Scalar, simulation sampling time [s]
%     i         - Integer, current simulation step (1-based)
%
%   Outputs:
%     systemNext - Cell array, updated system states at next step (with commands, controller, observer, etc.)
%
%   Usage Example:
%     systemNext = updateUsual(N, ASVs, system, conSystem, env, Ts, i)
%
%   Author: Wenxiang Wu
%   Date:   2025-05-14

systemNext=system;
for j=1:length(system)
    systemNext{j}.commands(i,:)=conSystem{j}.controller.commands(i,:);
    if isfield(ASVs{j},'observer')
        systemNext{j}.observer=conSystem{j}.observer;
    end
    if isfield(ASVs{j},'controller')
        systemNext{j}.controller=conSystem{j}.controller;
    end
    disturbance{j}.current=[0 0 0];
    disturbance{j}.force=[0 0 0];
end

if i<N   
    for j=1:length(system)
        states=system{j}.realStates(i,:);
        zz=dynamicsModel(ASVs{j}.dynamics,systemNext{j}.realStates(i,:)',systemNext{j}.commands(i,:)',disturbance{j});
        nextStates=states+Ts*zz';
        systemNext{j}.realStates(i+1,:)=nextStates;        
    end
end
end
