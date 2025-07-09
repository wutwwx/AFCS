function xd = graphBasedFormation(Trajectory,inputStates, A, Ship_num, task, taskNowNum,leaderNums)
%GRAPHBASEDFORMATION  Graph-theoretic distributed formation trajectory generation.
%
%   xd = graphBasedFormation(Trajectory, inputStates, A, Ship_num, task, taskNowNum, leaderNums)
%   computes the expected reference trajectory for each ship at the current time step,
%   based on a graph structure (adjacency matrix), formation geometry, and neighbor fusion.
%   This supports decentralized distributed formation following, leader/follower switching,
%   and multiple communication topologies.
%
%   Inputs:
%     Trajectory  - [T×3] array, global formation reference trajectory (center or leader trajectory)
%     inputStates - cell array, {Ship_num}, each is [K×6/8/...] actual state history of each ship (usually up to current step)
%     A           - [Ship_num×Ship_num] adjacency matrix (A(i,j)=1: i can receive from j)
%     Ship_num    - integer, total number of ships in formation
%     task        - struct, scenario description (fields: .geometry.distances, .geometry.angles, etc.)
%     taskNowNum  - integer, current task index
%     leaderNums  - vector, indices of leader ships for current task (from strategy.GB.leaderNums)
%
%   Outputs:
%     xd          - cell array, {Ship_num+1}, 
%                     xd{1}: global trajectory (copy of Trajectory),
%                     xd{j+1}: reference trajectory for each member ship (j = 1...Ship_num)
%
%   Usage Example:
%     xd = graphBasedFormation(Trajectory, inputStates, A, Ship_num, task, taskNowNum, leaderNums);
%
%   Reference:
%     [1] Park BS, Yoo SJ.
%         " An error transformation approach for connectivity-preserving and collision-avoiding formation tracking of networked uncertain underactuated surface vessels."
%          IEEE transactions on cybernetics. 2018 May 30;49(8):2955-66.
%
%   Author: Wenxiang Wu
%   Date:   2025-06-15

    xd{1} = Trajectory;
    leaders=leaderNums(taskNowNum,:);
    theta{1}=tangentAngleCalculate(xd{1});
    for j=1:Ship_num
        for i=1:size(inputStates{j},1)
            u_e{j}(i,1)=inputStates{j}(i,4)*cos(inputStates{j}(i,3))-inputStates{j}(i,5)*sin(inputStates{j}(i,3));
            v_e{j}(i,1)=inputStates{j}(i,4)*sin(inputStates{j}(i,3))+inputStates{j}(i,5)*cos(inputStates{j}(i,3));
%             if u_e{j}(i,1)==0 && v_e{j}(i,1)==0
%                 theta{j+1}(i,1)=inputStates{j}(i,3);
%             else
%                 theta{j+1}(i,1)= atan2(v_e{j}(i,1), u_e{j}(i,1));
%             end  
        theta{j+1}(i,1)=inputStates{j}(i,3);
        end  
        d_a(1,j)=task.geometry.distances(taskNowNum, j);
        a_a(1,j)=task.geometry.angles(taskNowNum, j);
    end
    for j=1:Ship_num
        neighbors = find(A(j, :) ~= 0);  
        if isempty(neighbors)
            error(['Ship ', num2str(j), ' has no neighbor in adjacency graph.']);
        end
        for i = neighbors
            x2 = task.geometry.distances(taskNowNum, i);
            p1 = task.geometry.angles(taskNowNum, j);
            p2 = task.geometry.angles(taskNowNum, i);
            d_r(j,i) = sqrt((d_a(1,j)*sin(a_a(1,j))-d_a(1,i)*sin(a_a(1,i)))^2 + (d_a(1,j)*cos(a_a(1,j))-d_a(1,i)*cos(a_a(1,i)))^2);
            a_r(j,i) = atan2(d_a(1,j)*sin(a_a(1,j))-d_a(1,i)*sin(a_a(1,i)), d_a(1,j)*cos(a_a(1,j))-d_a(1,i)*cos(a_a(1,i)));
        end
    end

    for j = 1:Ship_num
        if ismember(j,leaders)
            for i=1:size(Trajectory,1)
                xd{j+1}(i,1)=xd{1}(i,1)+d_a(1,j)*cos(theta{1}(i,1)+a_a(1,j));
                xd{j+1}(i,2)=xd{1}(i,2)+d_a(1,j)*sin(theta{1}(i,1)+a_a(1,j));
                xd{j+1}(i,3)=xd{1}(i,3);
            end   
        else
            neighbors = find(A(j, :) ~= 0);  

            xdSize=0;
            for i =neighbors
                xdSize=max(xdSize,size(inputStates{i},1));
            end
            xd{j+1}=zeros(xdSize,3);           
            for k = 1:xdSize
                valid_count = 0;

                for i = neighbors
                    if size(inputStates{i}, 1) < k
                        continue  
                    end                     
                    xd{j+1}(k,1)=xd{j+1}(k,1)+(inputStates{i}(k,1)+d_r(j,i)*cos(theta{i+1}(k,1)+a_r(j,i)));
                    xd{j+1}(k,2)=xd{j+1}(k,2)+(inputStates{i}(k,2)+d_r(j,i)*sin(theta{i+1}(k,1)+a_r(j,i)));
                    xd{j+1}(k,3)=xd{j+1}(k,3)+(inputStates{i}(k,3));

                    valid_count = valid_count + 1;
                end

                    xd{j+1}(k,1:3) = xd{j+1}(k,1:3) / valid_count;
            end                      
        end
    end
end
