function ship = cybership2()
%CYBERSHIP2FOSSEN  Returns the parameter structure for the NTNU CyberShip II vessel model.
%
%   ship = cybership2Fossen() returns a structure containing the physical 
%   and hydrodynamic parameters for the CyberShip II (NTNU, Fossen) 3-DOF model.
%   These parameters are commonly used for simulation and control research 
%   involving small-scale surface vessels.
%
%   The structure fields include:
%     ship.type     : Model type indicator (1 = Fossen 3-DOF)
%     ship.m, mass  : Vessel mass (kg)
%     ship.L        : Vessel length (m)
%     ship.B        : Vessel beam/width (m)
%     ship.xg       : x-coordinate of center of gravity (m)
%     ship.Iz       : Yaw moment of inertia (kg·m^2)
%     ship.m11, m22, m33, m23, m32 : Added mass matrix components
%     ship.Xu, Xuu, Xuuu           : Hydrodynamic surge damping coefficients
%     ship.Yv, Yvv, Yr, Yrv, Yvr, Yrr : Sway and yaw hydrodynamic coefficients
%     ship.Nv, Nvv, Nr, Nrv, Nvr, Nrr : Yaw hydrodynamic coefficients
%     ship.M     : 3×3 inertia matrix (incl. added mass)
%     ship.maxtau, mintau : Control input saturation limits
%     ship.Nx    : State dimension (default 6)
%     ship.Nu    : Input dimension (default 3)
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
%     ship = cybership2Fossen;
%     disp(ship)
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

    ship.type=1;
    ship.mass=23.8;
    ship.m=23.8;
    ship.L=1.255;
    ship.B = 0.29;
    ship.xg=0.046;
    ship.Iz=1.76;
    ship.Xdot_u=-2.0;
    ship.Ydot_v=-10.0;        
    ship.Ydot_r=-0.0;
    ship.Ndot_v=-0.0;        
    ship.Ndot_r=-1.0;
    ship.m11=ship.m-ship.Xdot_u; 
    ship.m22=ship.m-ship.Ydot_v;  
    ship.m23=ship.m*ship.xg-ship.Ydot_r; 
    ship.m32=ship.m*ship.xg-ship.Ndot_v; 
    ship.m33=ship.Iz-ship.Ndot_r; 
    ship.Xu=-0.72253; 
    ship.Xuu=-1.32742; 
    ship.Xuuu=-5.86643;
    ship.Yv=-0.88965; 
    ship.Yvv=-36.47287;
    ship.Yr=-7.250; 
    ship.Yrv=-0.805; 
    ship.Yvr=-0.845; 
    ship.Yrr=-3.45; 
    ship.Nv=0.03130; 
    ship.Nvv=3.95645; 
    ship.Nr=-1.9; 
    ship.Nrv=0.13; 
    ship.Nvr=0.08; 
    ship.Nrr=-0.75;
    ship.M=[
            ship.m11  0    0
            0   ship.m22  ship.m23
            0   ship.m32  ship.m33
            ];
    ship.maxtau=[2,2,1.5];
    ship.mintau=[-2,-2,-1.5];
    ship.Nx = 6; 
    ship.Nu = 3;
    ship.MaxInputs=[2; 1.5; 1.5];
    ship.MinInputs=[-2; -1.5; -1.5];

end
