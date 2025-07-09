function FormationTrajectory=ForPathToTra(task,N,Ts)
%FORPATHTOTRA   Convert task path/trajectory definitions to unified trajectory arrays for simulation.
%
%   FormationTrajectory = ForPathToTra(task, N, Ts)
%   Converts the 'task' structure (with PathFollowing or TrajectoryTracking tasks)
%   into time-discretized target trajectories for each mission stage.
%
%   Inputs:
%     task   - Struct, mission definition (fields: .name, .target, .time)
%     N      - Integer, total simulation steps
%     Ts     - Scalar, simulation sampling time (s)
%
%   Output:
%     FormationTrajectory - Cell array, each cell is a [M×3] trajectory (x, y, psi)
%
%   Usage Example:
%     FormationTrajectory = ForPathToTra(task, ,N,Ts)
%
%   Author: Wenxiang Wu
%   Date:   2025-07-05

taskNum=length(task.time);
for i =1:taskNum
    if task.name(1,i)=="Reconfiguration"
        task.name(1,i)=task.name(1,i-1);
        task.target{i}=task.target{i-1};
        task.time(1,i)=task.time(1,i-1);
    end
    switch task.name(1,i)
            case "PathFollowing" 
                FormationPath=task.target{i}.path; 
                FormationTrajectory{i}=resampleCurve(FormationPath, Ts*task.target{i}.velocity);
                if i<taskNum
                    if size(FormationTrajectory{i},1)<(task.time(1,i+1)-task.time(1,i))/Ts
                        error(['The number of target trajectory points for the ' num2str(i) 'th task is less than the required execution steps for the task.']);
                    end
                else
                    if size(FormationTrajectory{i},1)<(N-task.time(1,i))/Ts
                        error(['The number of target trajectory points for the ' num2str(i) 'th task is less than the required execution steps for the task.']);
                    end   
                end 
            case "TrajectoryTracking"
                FormationTrajectory{i}=task.target{i}.trajectory; 
                if i<taskNum
                    if size(FormationTrajectory{i},1)<(task.time(1,i+1)-task.time(1,i))/Ts
                        error(['The number of target trajectory points for the ' num2str(i) 'th task is less than the required execution steps for the task.']);
                    end
                else
                    if size(FormationTrajectory{i},1)<(N-task.time(1,i))/Ts
                        error(['The number of target trajectory points for the ' num2str(i) 'th task is less than the required execution steps for the task.']);
                    end   
                end                                
    end
end
