function ASV = cybership2()
%CYBERASV2FOSSEN  Returns the parameter structure for the NTNU CyberShip II vessel model.
%
%   ASV = cybership2Fossen() returns a structure containing the physical 
%   and hydrodynamic parameters for the CyberShip II (NTNU, Fossen) 3-DOF model.
%   These parameters are commonly used for simulation and control research 
%   involving small-scale surface vessels.
%
%   The structure fields include:
%     ASV.type     : Model type indicator (1 = Fossen 3-DOF)
%     ASV.m, mass  : Vessel mass (kg)
%     ASV.L        : Vessel length (m)
%     ASV.B        : Vessel beam/width (m)
%     ASV.xg       : x-coordinate of center of gravity (m)
%     ASV.Iz       : Yaw moment of inertia (kg·m^2)
%     ASV.m11, m22, m33, m23, m32 : Added mass matrix components
%     ASV.Xu, Xuu, Xuuu           : Hydrodynamic surge damping coefficients
%     ASV.Yv, Yvv, Yr, Yrv, Yvr, Yrr : Sway and yaw hydrodynamic coefficients
%     ASV.Nv, Nvv, Nr, Nrv, Nvr, Nrr : Yaw hydrodynamic coefficients
%     ASV.M     : 3×3 inertia matrix (incl. added mass)
%     ASV.maxtau, mintau : Control input saturation limits
%     ASV.Nx    : State dimension (default 6)
%     ASV.Nu    : Input dimension (default 3)
%     % Wind parameters (set to 0 by default)
%
%   Reference:
%     T.I. Fossen (2011). "Handbook of Marine Craft Hydrodynamics and Motion Control,"
%     Wiley, Section 9.4, Table 9.2. (CyberShip II model)
%
%   Author: Wenxiang Wu
%   Date:   2025-06-30
%
%   Example usage:
%     ASV = cybership2Fossen;
%     disp(ASV)
%
% -------------------------------------------------------------------------
%  State vector (for typical use with 3-DOF model):
%    [x, y, psi, u, v, r]
%      x   : North position (m)
%      y   : East position (m)
%      psi : Yaw angle (rad)
%      u   : Surge velocity (m/s)
%      v   : Sway velocity (m/s)
%      r   : Yaw rate (rad/s)
%
% -------------------------------------------------------------------------

    ASV.type=1;
    ASV.mass=23.8;
    ASV.m=23.8;
    ASV.L=1.255;
    ASV.B = 0.29;
    ASV.xg=0.046;
    ASV.Iz=1.76;
    ASV.Xdot_u=-2.0;
    ASV.Ydot_v=-10.0;        
    ASV.Ydot_r=-0.0;
    ASV.Ndot_v=-0.0;        
    ASV.Ndot_r=-1.0;
    ASV.m11=ASV.m-ASV.Xdot_u; 
    ASV.m22=ASV.m-ASV.Ydot_v;  
    ASV.m23=ASV.m*ASV.xg-ASV.Ydot_r; 
    ASV.m32=ASV.m*ASV.xg-ASV.Ndot_v; 
    ASV.m33=ASV.Iz-ASV.Ndot_r; 
    ASV.Xu=-0.72253; 
    ASV.Xuu=-1.32742; 
    ASV.Xuuu=-5.86643;
    ASV.Yv=-0.88965; 
    ASV.Yvv=-36.47287;
    ASV.Yr=-7.250; 
    ASV.Yrv=-0.805; 
    ASV.Yvr=-0.845; 
    ASV.Yrr=-3.45; 
    ASV.Nv=0.03130; 
    ASV.Nvv=3.95645; 
    ASV.Nr=-1.9; 
    ASV.Nrv=0.13; 
    ASV.Nvr=0.08; 
    ASV.Nrr=-0.75;
    ASV.M=[
            ASV.m11  0    0
            0   ASV.m22  ASV.m23
            0   ASV.m32  ASV.m33
            ];
    ASV.maxtau=[2,2,1.5];
    ASV.mintau=[-2,-2,-1.5];
    ASV.Nx = 6; 
    ASV.Nu = 3;
    ASV.MaxInputs=[2; 1.5; 1.5];
    ASV.MinInputs=[-2; -1.5; -1.5];

end
