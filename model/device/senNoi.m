function SenNoise=senNoi(shipNum,Nx,N)
%SENNOI   Generate measurement noise samples for ship sensor simulation.
%
%   SenNoise = senNoi(shipNum, Nx, N) creates a cell array of Gaussian
%   measurement noise vectors for each ship and each sensor dimension,
%   suitable for adding to simulated sensor readings in multi-ship scenarios.
%
%   Inputs:
%     shipNum    : Number of ships
%     Nx         : [1 × shipNum] vector, state/sensor dimension per ship
%     N          : Number of time steps (noise samples per sensor)
%
%   Outputs:
%     SenNoise   : Cell array (1×shipNum), each cell is an [N × Nx(j)] matrix 
%                  of independent Gaussian noise samples (default: std=0.1)
%
%   Description:
%     - Generates zero-mean Gaussian noise (std = 0.1 by default) for each 
%       ship and each sensor (state) dimension, for N time steps.
%     - Can be used to simulate sensor reading noise in state estimation, filtering, or control.
%     - For custom noise statistics, modify the scale factor in the code.
%
%   Author: Wenxiang Wu
%   Date:   2025-03-30
%
%   Example usage:
%     SenNoise = senNoi(shipNum, Nx, N);

SenNoise=cell(1,shipNum);

for j=1:shipNum
    for i=1:Nx(1,j)
        SenNoise{j}(:,i)=0.1*randn(N,1); 
    end
end
