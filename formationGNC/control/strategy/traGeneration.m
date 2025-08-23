function [ForTarTra,memTarTra]=traGeneration(task,FormationTrajectories,strategy,systemStates,GNCStates,Ts,step)
%TRAGENERATION  Generate reference trajectories for formation ASVs under multi-strategy, multi-task scenarios.
%
%   [ForTarTra, memTarTra] = traGeneration(task, FormationTrajectories, strategy, systemStates, GNCStates, Ts, step)
%   computes, at each simulation time step, the target trajectories for all ASVs
%   under the selected formation strategy (VirtualStructure / LeaderFollower / GraphBased).
%   Multi-segment, multi-task, multi-ASV switching is supported. 
%
%   Inputs:
%     task                  - Struct, scenario information (fields: .time, .name, .geometry)
%     FormationTrajectories - Cell array, {taskNum}, each is [T×3] formation reference path for each task
%     strategy              - Struct, formation strategy settings (see strategySetting output)
%     systemStates          - Cell array, {ASV_num}, with .realStates, actual ASV states at each step
%     GNCStates             - Cell array, {ASV_num}, GNC/Controller prediction info (may be empty)
%     Ts                    - Scalar, simulation time step [s]
%     step                  - Scalar, current simulation step (integer)
%
%   Outputs:
%     ForTarTra    - [M×3] array, global reference trajectory for the formation (center/reference)
%     memTarTra    - Cell array, {ASV_num}, each [M×3], the reference trajectory for each ASV (member ASV)
%
%   Usage Example:
%     [ForTarTra, memTarTra] = traGeneration(task, FormationTrajectories, strategy, systemStates, GNCStates, Ts, step)
%
%   Author: Wenxiang Wu
%   Date:   2025-06-11

taskNum=length(task.time);
tt=Ts*(step-1);
for i=1:taskNum
    if tt>=task.time(1,taskNum-i+1)
        taskNowNum=taskNum-i+1;
        taskNow=task.name(1,taskNowNum);
        break
    end
end                   
FormationTrajectory=FormationTrajectories{taskNowNum};
ASV_num=size(task.geometry.distances,2);
switch strategy.name
    case "VirtualStructure"
        if taskNow=="Reconfiguration"
            Trajectory=FormationTrajectory(step-task.time(1,taskNowNum-1):size(FormationTrajectory,1),:);
        else
            Trajectory=FormationTrajectory(step-task.time(1,taskNowNum):size(FormationTrajectory,1),:);
        end
        xd = virtualStructure(Trajectory,ASV_num,task,taskNowNum);          
    case "LeaderFollower"
        if taskNow=="Reconfiguration"
            Trajectory=FormationTrajectory(step-task.time(1,taskNowNum-1):size(FormationTrajectory,1),:);
        else
            Trajectory=FormationTrajectory(step-task.time(1,taskNowNum):size(FormationTrajectory,1),:);
        end
        for j=1:ASV_num
            if ~isempty(GNCStates)
                if isfield(GNCStates{j}.controller,"preStates")
                    inputStates{j}= GNCStates{j}.controller.preStates;
                end  
            end
            inputStates{j}(1,:)=systemStates{j}.realStates(step,:);
        end
        xd = leaderFollower(Trajectory,inputStates,ASV_num,task,taskNowNum,strategy.LF.refNums);
    case "GraphBased"
        if taskNow=="Reconfiguration"
            Trajectory=FormationTrajectory(step-task.time(1,taskNowNum-1):size(FormationTrajectory,1),:);
        else
            Trajectory=FormationTrajectory(step-task.time(1,taskNowNum):size(FormationTrajectory,1),:);
        end
        for j=1:ASV_num     
            if ~isempty(GNCStates)
                if isfield(GNCStates{j}.controller,"preStates")
                    inputStates{j}= GNCStates{j}.controller.preStates;
                end  
            end
            inputStates{j}(1,:)=systemStates{j}.realStates(step,:);
        end        
        Adjacency=buildAdjacencyByDistance(inputStates, taskNowNum,strategy.GB.recv_radius,strategy.GB.send_radius);
        xd = graphBasedFormation(Trajectory,inputStates,Adjacency, ASV_num, task, taskNowNum,strategy.GB.leaderNums);
end

ForTarTra=xd{1};
for i=1:size(ForTarTra,1)-1
    if ForTarTra(i+1,3)-ForTarTra(i,3)<-pi
        ForTarTra(i+1,3)=ForTarTra(i+1,3)+2*pi;
    elseif ForTarTra(i+1,3)-ForTarTra(i,3)>pi
        ForTarTra(i+1,3)=ForTarTra(i+1,3)-2*pi;
    else
        ForTarTra(i+1,3)=ForTarTra(i+1,3);
    end
end  
for j=1:ASV_num
    memTarTra{j}= xd{j+1};
    for i=1:size(memTarTra{j},1)-1
        if memTarTra{j}(i+1,3)-memTarTra{j}(i,3)<-pi
            memTarTra{j}(i+1,3)=memTarTra{j}(i+1,3)+2*pi;
        elseif memTarTra{j}(i+1,3)-memTarTra{j}(i,3)>pi
            memTarTra{j}(i+1,3)=memTarTra{j}(i+1,3)-2*pi;
        else
            memTarTra{j}(i+1,3)=memTarTra{j}(i+1,3);
        end
    end    
end               
end
