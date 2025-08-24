function ASV=yunfan1()
%YUNFAN1   Returns MMG model parameters for the 'Yunfan1' ASV (WUT, Renjiantianjin prototype).
%
%   ASV = yunfan1()
%   Outputs a struct containing the physical, geometric, and hydrodynamic coefficients
%   required for MMG maneuvering simulation of Yunfan1, a single-propeller, single-rudder model ASV.
%
%   Yunfan1 is a scaled ASV model at the State Key Laboratory of Maritime Technology and Safety,
%   Wuhan University of Technology. The prototype is the "Renjiantianjin" vessel.
%
%   Output:
%     ASV   - Struct of MMG parameters (see field list below)
%
%   Reference:
%     Introduction of MMG standard method for ASV maneuvering predictions
%
%   Author: Wenxiang Wu
%   Date:   2025-02-19

    ASV.rho=998.1;
    ASV.type=2;
    ASV.L=1.66;
    ASV.d=0.105;
    ASV.delta_max=35*pi/180;
    ASV.Delta_max=40*pi/180;
    ASV.n_max=4000;
    ASV.B=0.276;
    ASV.mass=33.9;   
    ASV.m=0.1543;
    ASV.mx=0.067;
    ASV.my=0.1521;
    ASV.Jzz=0.179;
    ASV.Izz=0.005236;
    ASV.Xuu=-0.01948;
    ASV.Xvv=-0.09996;
    ASV.Xvr=-0.0318;
    ASV.Xrr=-0.02072;      
    ASV.Yv=-0.3536;
    ASV.Yr=0.01515;
    ASV.Yvvv=-1.13;
    ASV.Yvvr=-0.2821;
    ASV.Yvrr=-0.2057;
    ASV.Yrrr=0.007611;
    ASV.Nv=-0.3034;
    ASV.Nr=-0.2104;
    ASV.Nvvv=-0.02292;
    ASV.Nvvr=-0.10824;
    ASV.Nvrr=0.05782;
    ASV.Nrrr=0.0016042;
    ASV.tR=0.2044;
    ASV.aH=0.7269;
    ASV.xH=-0.5143;
    ASV.xR=-0.5;
    ASV.AR=0.00413;
    ASV.Delta=1.056;
    ASV.P = [3.2971, -1.0554, 0.2079];
    ASV.epsilon=1.2619;        
    ASV.kk=0.633;
    ASV.uP=0.645;
    ASV.tP=0.257;
    ASV.gammaR1=0.492;
    ASV.gammaR2=0.338;
    ASV.DP=0.051;
    ASV.lR=-0.755;
    ASV.eta=0.822;
    ASV.Nx = 8; 
    ASV.Nu = 2;
    ASV.KT=0.2712;
    ASV.J1=0.3502;
    ASV.MinInputs=[-35*pi/180;  -3000/60];
    ASV.MaxInputs=[35*pi/180;  3000/60];
end
