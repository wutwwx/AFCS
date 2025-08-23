function xd = leaderFollower(Trajectory,inputStates,ASV_num,task,taskNowNum,refNums) 
%LEADERFOLLOWER  Leader-Follower formation reference trajectory generation.
%
%   xd = leaderFollower(Trajectory, inputStates, ASV_num, task, taskNowNum, refNums)
%   computes the expected reference trajectory for each ASV based on the
%   leader-follower topology. Each ASV either follows the global formation trajectory
%   (if refNums == 0) or follows another ASV (if refNums(j) == k, j ≠ k), with
%   geometry defined by inter-ASV distances and angles.
%
%   Inputs:
%     Trajectory   - [T×3] array, global (formation center or leader) trajectory
%     inputStates  - cell array, {ASV_num}, each is [K×n] actual state history of each ASV
%     ASV_num     - integer, total number of ASVs in formation
%     task         - struct, scenario/geometry info (fields: .geometry.distances, .geometry.angles, etc.)
%     taskNowNum   - integer, index for current task/segment
%     refNums      - [1×ASV_num] vector, leader index for each ASV at current task (0: follows global; j: follows ASV j)
%
%   Outputs:
%     xd           - cell array, {ASV_num+1}, xd{1} is global, xd{j+1} is reference for each ASV
%
%   Usage Example:
%     xd = leaderFollower(Trajectory, inputStates, ASV_num, task, taskNowNum, refNums);
%
%   Reference:
%     [1] Sun Z, Zhang G, Lu Y, Zhang W.
%         " Leader-follower formation control of underactuated surface vehicles based on sliding mode control and parameter estimation. "
%          ISA transactions. 2018 Jan 1;72:15-24.
%
%   Author: Wenxiang Wu
%   Date:   2025-04-09

    xd{1}=Trajectory;
    refs=refNums(taskNowNum,:);
    if length(refs)~=ASV_num    
        error('The number of ASV references is not consistent with the number of ASVs');
    end

    for j=1:ASV_num 
        if refs(1,j)==j
            error('The ASV in formation cannot set its own position as a reference object');
        end
        if refs(1,j)~=0
            if refs(1,refs(1,j))==j
                error('The Leader-Follower should not allow two ASVs to use each other positions as reference object');
            end
        end
    end
    if prod(refs)~=0
        error('At least one ASV needs to take the formation trajectory as the reference object');
    end
    theta{1}=tangentAngleCalculate(xd{1});
    for j=1:ASV_num             
        if refs(1,j)==0            
            distances=task.geometry.distances(taskNowNum,j);% Calculate the distance from the reference ASV.
            angles=task.geometry.angles(taskNowNum,j); % Calculate the angle relative to the reference ASV.
            for i=1:size(Trajectory,1)
                xd{j+1}(i,1)=xd{1}(i,1)+distances*cos(theta{1}(i,1)+angles);
                xd{j+1}(i,2)=xd{1}(i,2)+distances*sin(theta{1}(i,1)+angles);
                xd{j+1}(i,3)=xd{1}(i,3);
            end   
        else                   
            x1=task.geometry.distances(taskNowNum,j);
            x2=task.geometry.distances(taskNowNum,refs(1,j));
            p1=task.geometry.angles(taskNowNum,j);
            p2=task.geometry.angles(taskNowNum,refs(1,j));
            distances=sqrt((x1*sin(p1)-x2*sin(p2))^2+(x1*cos(p1)-x2*cos(p2))^2); % Calculate the distance from the reference ASV.
            angles=atan2(x1*sin(p1)-x2*sin(p2),x1*cos(p1)-x2*cos(p2)); % Calculate the angle relative to the reference ASV. 
            for i=1:size(inputStates{refs(1,j)},1)
                u_e{refs(1,j)}(i,1)=inputStates{refs(1,j)}(i,4)*cos(inputStates{refs(1,j)}(i,3))-inputStates{refs(1,j)}(i,5)*sin(inputStates{refs(1,j)}(i,3));
                v_e{refs(1,j)}(i,1)=inputStates{refs(1,j)}(i,4)*sin(inputStates{refs(1,j)}(i,3))+inputStates{refs(1,j)}(i,5)*cos(inputStates{refs(1,j)}(i,3));
%                 if u_e{refs(1,j)}(i,1)==0 && v_e{refs(1,j)}(i,1)==0
%                     theta{refs(1,j)+1}(i,1)=inputStates{refs(1,j)}(i,3);
%                 else
%                     theta{refs(1,j)+1}(i,1)= atan2(v_e{refs(1,j)}(i,1), u_e{refs(1,j)}(i,1));
%                 end   
                theta{refs(1,j)+1}(i,1)=inputStates{refs(1,j)}(i,3);
                xd{j+1}(i,1)=inputStates{refs(1,j)}(i,1)+distances*cos(theta{refs(1,j)+1}(i,1)+angles);
                xd{j+1}(i,2)=inputStates{refs(1,j)}(i,2)+distances*sin(theta{refs(1,j)+1}(i,1)+angles);
                xd{j+1}(i,3)=inputStates{refs(1,j)}(i,3);                        
            end
        end
    end  
end

