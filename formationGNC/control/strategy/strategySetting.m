function strategyparas=strategySetting(name,task,shipNum)
%STRATEGYSETTING  Set up formation control strategy parameters.
%
%   strategyparas = strategySetting(name, task, shipNum)
%   returns a struct with all parameters required for a specific formation
%   strategy (e.g., Leader-Follower or Graph-Based), with task-dependent and
%   ship-dependent configurations automatically expanded for multi-task simulation.
%
%   Inputs:
%     name    : String, formation strategy name, e.g., 'LeaderFollower', 'GraphBased'
%     task    : Struct, with field 'name', representing a list of scenario/task names
%     shipNum : Integer, number of ships in the formation
%
%   Output:
%     strategyparas : Struct, includes strategy name and required per-task, per-ship settings,
%                     e.g. for 'LeaderFollower': .LF.refNums, for 'GraphBased': .GB.leaderNums,
%                     .GB.recv_radius, .GB.send_radius (all expanded to task×shipNum arrays)
%
%   Usage Example:
%     task.name = {'task1','task2'};
%     strategyparas = strategySetting('GraphBased', task, shipNum);
%
%   Author: Wenxiang Wu
%   Date:   2025-06-11

strategyparas.name=name;
switch name
    case "LeaderFollower"
        refNums=[ 0, 1, 1, 1]; %The identification number of the leader of each follower.
        if size(refNums,1)==1
            refNums = repmat(refNums, length(task.name), 1);
        elseif length(task.name)~=size(refNums,1)
            error('The number of tasks does not equal the number of refer settings for LeaderFollower.')
        end
        if size(refNums,2)==1
            refNums = repmat(refNums, 1, shipNum);
        elseif size(refNums,2)~=shipNum
            error('There are ships in the formation that have not set a reference object.')
        end
        strategyparas.LF.refNums=refNums;
    case "GraphBased"
        leaderNums=1; 
        recv_radius=30;
        send_radius=30;       
        if size(leaderNums,1)==1
            leaderNums = repmat (leaderNums, length(task.name), 1);
        elseif length(task.name)~=size(leaderNums,1)
            error('The number of tasks does not equal the number of leader settings for GraphBased.')
        end
        strategyparas.GB.leaderNums=leaderNums;
        
        if size(recv_radius,1)==1
            recv_radius = repmat(recv_radius, length(task.name), 1);
        elseif length(task.name)~=size(recv_radius,1)
            error('The number of tasks does not equal the number of receive radius settings for GraphBased.')
        end
        if size(recv_radius,2)==1
            recv_radius = repmat(recv_radius, 1, shipNum);
        elseif shipNum~=size(recv_radius,2)
            error('There are ships in the formation that have not set a receive radius for GraphBased.')
        end
        strategyparas.GB.recv_radius=recv_radius;
        
        if size(send_radius,1)==1
            send_radius = repmat(send_radius, length(task.name), 1);
        elseif length(task.name)~=size(send_radius,1)
            error('The number of tasks does not equal the number of send radius settings for GraphBased.')
        end
        if size(send_radius,2)==1
            send_radius = repmat(send_radius, 1, shipNum);
        elseif shipNum~=size(send_radius,2)
            error('There are ships in the formation that have not set a send radius for GraphBased.')
        end       
        strategyparas.GB.send_radius=send_radius;
        
end

end