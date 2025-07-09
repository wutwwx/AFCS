function ship=yunfan1()
%YUNFAN1   Returns MMG model parameters for the 'Yunfan1' ship (WUT, Renjiantianjin prototype).
%
%   ship = yunfan1()
%   Outputs a struct containing the physical, geometric, and hydrodynamic coefficients
%   required for MMG maneuvering simulation of Yunfan1, a single-propeller, single-rudder model ship.
%
%   Yunfan1 is a scaled ship model at the State Key Laboratory of Maritime Technology and Safety,
%   Wuhan University of Technology. The prototype is the "Renjiantianjin" vessel.
%
%   Output:
%     ship   - Struct of MMG parameters (see field list below)
%
%   Reference:
%     Introduction of MMG standard method for ship maneuvering predictions
%
%   Author: Wenxiang Wu
%   Date:   2025-02-19

    ship.rho=998.1;
    ship.type=2;
    ship.L=1.66;
    ship.d=0.105;
    ship.delta_max=35*pi/180;
    ship.Delta_max=40*pi/180;
    ship.n_max=4000;
    ship.B=0.276;
    ship.mass=33.9;   
    ship.m=0.1543;
    ship.mx=0.067;
    ship.my=0.1521;
    ship.Jzz=0.179;
    ship.Izz=0.005236;
    ship.Xuu=-0.01948;
    ship.Xvv=-0.09996;
    ship.Xvr=-0.0318;
    ship.Xrr=-0.02072;      
    ship.Yv=-0.3536;
    ship.Yr=0.01515;
    ship.Yvvv=-1.13;
    ship.Yvvr=-0.2821;
    ship.Yvrr=-0.2057;
    ship.Yrrr=0.007611;
    ship.Nv=-0.3034;
    ship.Nr=-0.2104;
    ship.Nvvv=-0.02292;
    ship.Nvvr=-0.10824;
    ship.Nvrr=0.05782;
    ship.Nrrr=0.0016042;
    ship.tR=0.2044;
    ship.aH=0.7269;
    ship.xH=-0.5143;
    ship.xR=-0.5;
    ship.AR=0.00413;
    ship.Delta=1.056;
    ship.P = [3.2971, -1.0554, 0.2079];
    ship.epsilon=1.2619;        
    ship.kk=0.633;
    ship.uP=0.645;
    ship.tP=0.257;
    ship.gammaR1=0.492;
    ship.gammaR2=0.338;
    ship.DP=0.051;
    ship.lR=-0.755;
    ship.eta=0.822;
    ship.Nx = 8; 
    ship.Nu = 2;
    ship.KT=0.2712;
    ship.J1=0.3502;
    ship.MinInputs=[-35*pi/180;  -3000/60];
    ship.MaxInputs=[35*pi/180;  3000/60];
end