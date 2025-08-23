function xd = virtualStructure(Trajectory,ASV_num,task,taskNowNum)
%VIRTUALSTRUCTURE  Virtual Structure formation reference trajectory generation.
%
%   xd = virtualStructure(Trajectory, ASV_num, task, taskNowNum)
%   generates reference trajectories for all ASVs in the formation using the
%   virtual structure method, where all ASVs maintain a fixed geometric relation
%   to the formation center. The orientation at each point is given by the path tangent.
%
%   Inputs:
%     Trajectory    - [T×3] matrix, reference trajectory for the formation center
%     ASV_num      - Integer, number of ASVs in the formation
%     task          - Struct, task/geometry configuration (contains geometry.distances, geometry.angles)
%     taskNowNum    - Integer, current task index
%
%   Outputs:
%     xd            - Cell array, {ASV_num+1}, xd{1} is the center (global), xd{j+1} is ASV j’s trajectory
%
%   Usage Example:
%     xd = virtualStructure(Trajectory, ASV_num, task, taskNowNum);
%
%   Reference:
%     [1] Lewis MA, Tan KH.
%         " High precision formation control of mobile robots using virtual structures."
%          Autonomous robots. 1997 Oct;4:387-403.
%
%   Author: Wenxiang Wu
%   Date:   2025-03-11

    xd{1}=Trajectory;
    theta{1}=tangentAngleCalculate(xd{1});            
    for j=1:ASV_num
        distances(1,j)=task.geometry.distances(taskNowNum,j);% Calculate the distance from the reference ASV.
        angles(1,j)=task.geometry.angles(taskNowNum,j); % Calculate the angle relative to the reference ASV.
        for i=1:size(Trajectory,1)
            xd{j+1}(i,1)=xd{1}(i,1)+distances(1,j)*cos(theta{1}(i,1)+angles(1,j));
            xd{j+1}(i,2)=xd{1}(i,2)+distances(1,j)*sin(theta{1}(i,1)+angles(1,j));
            xd{j+1}(i,3)=xd{1}(i,3);
        end   
        theta_2{j+1} = tangentAngleCalculate(xd{j+1});
        xd{j+1}(:,3) = theta_2{j+1};
    end   
end

