function A = buildAdjacencyByDistance(inputStates, taskNowNum,receives, sends)
%BUILDADJACENCYBYDISTANCE  Construct directed adjacency matrix for multi-ship communication topology.
%
%   A = buildAdjacencyByDistance(inputStates, taskNowNum, receives, sends)
%   builds a directed adjacency matrix A (N×N) for a multi-ship (or multi-agent) network,
%   using the dual-radius communication model. Each ship (agent) is a node; an edge A(i,j)=1
%   indicates ship i can receive messages from ship j (i.e., ship j is in ship i's receive range
%   AND ship i is in ship j's send range).
%
%   Inputs:
%     inputStates : N×1 cell array. Each cell is a state vector for ship j;
%                   inputStates{j}(1,1:2) gives position (x,y) of ship j.
%     taskNowNum  : Integer, current task/time index (used to select radii).
%     receives    : T×N array, each row is the receive range of N ships at a time/task step.
%     sends       : T×N array, each row is the send range of N ships at a time/task step.
%
%   Output:
%     A           : N×N adjacency matrix (binary/directed). A(i,j)=1 iff
%                   ship i can receive from ship j at current step.
%
%   Example:
%       % inputStates{j}(1:2) = [x, y] of ship j
%       % receives/sends: e.g., ones(T,N) * 10 for a 10-meter radius for all
%       A = buildAdjacencyByDistance(inputStates, taskNowNum, receives, sends);
%
%   Author: Wenxiang Wu
%   Date:   2025-05-11

    N = length(inputStates);

    % --- Expand scalars to vectors ---
    recv_radius=receives(taskNowNum,:);
    send_radius=sends(taskNowNum,:);

    % --- Check input validity ---
    if length(recv_radius) ~= N || length(send_radius) ~= N
        error('recv_radius and send_radius must both be scalars or N×1 vectors.');
    end

    % --- Initialize adjacency matrix ---
    A = zeros(N, N);

    % --- Construct directed edges based on distance and dual-radius rule ---
    for i = 1:N        % Receiver (ship i)
        for j = 1:N    % Transmitter (ship j)
            if i ~= j
                dist_ij = norm(inputStates{i}(1,1:2) - inputStates{j}(1,1:2));
                if dist_ij <= recv_radius(1,i) && dist_ij <= send_radius(1,j)
                    A(i,j) = 1;  % i can receive from j
                end
            end
        end
    end
end
