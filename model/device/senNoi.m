function SenNoise=senNoi(ASVNum,Nx,N)
%SENNOI   Generate measurement noise samples for ASV sensor simulation.
%
%   SenNoise = senNoi(ASVNum, Nx, N) creates a cell array of Gaussian
%   measurement noise vectors for each ASV and each sensor dimension,
%   suitable for adding to simulated sensor readings in multi-ASV scenarios.
%
%   Inputs:
%     ASVNum    : Number of ASVs
%     Nx         : [1 × ASVNum] vector, state/sensor dimension per ASV
%     N          : Number of time steps (noise samples per sensor)
%
%   Outputs:
%     SenNoise   : Cell array (1×ASVNum), each cell is an [N × Nx(j)] matrix 
%                  of independent Gaussian noise samples (default: std=0.1)
%
%   Description:
%     - Generates zero-mean Gaussian noise (std = 0.1 by default) for each 
%       ASV and each sensor (state) dimension, for N time steps.
%     - Can be used to simulate sensor reading noise in state estimation, filtering, or control.
%     - For custom noise statistics, modify the scale factor in the code.
%
%   Author: Wenxiang Wu
%   Date:   2025-03-30
%
%   Example usage:
%     SenNoise = senNoi(ASVNum, Nx, N);

SenNoise=cell(1,ASVNum);

for j=1:ASVNum
    for i=1:Nx(1,j)
        SenNoise{j}(:,i)=0.1*randn(N,1); 
    end
end
