function observer = disObsYang(xiInputs,observer,commands,ship,Ts)
%DISOBSYANG  Nonlinear disturbance observer (Yang's method) for ship model.
%
%   observer = disObsYang(xiInputs, observer, commands, ship, Ts)
%   updates the disturbance estimate for a single ship based on nonlinear
%   model dynamics and the current/past states, using the observer design
%   from Yang et al.
%
%   Inputs:
%     xiInputs   - Struct, must contain:
%                   .statesNow  [1×n] current state [x y psi u v r ...]
%                   .statesPast [1×n] previous state [x y psi u v r ...]
%     observer   - Struct, must contain at least:
%                   .Belta     [3×1] internal observer state
%                   .K_omega   [scalar/3×3] observer gain
%     commands   - [1×3] control input vector at current step
%     ship       - Struct, must contain .dynamics (with full parameter set)
%     Ts         - Scalar, sampling time (s)
%
%   Outputs:
%     observer   - Struct, updated, with fields:
%                   .UnDis     [1×3] estimated unknown disturbance (tau_x, tau_y, tau_psi)
%                   .Belta     [3×1] updated observer state
%
%   Usage Example:
%     observer = disObsYang(xiInputs, observer, commands, ship, Ts)
%
%   Reference:
%     [1] C. Yang, L. Zhang, et al., "A Trajectory Tracking Robust Controller of
%         Surface Vessels With Disturbance Uncertainties," IEEE Trans. Ind. Electron.,
%         vol. 66, no. 12, pp. 9619–9630, Dec. 2019.
%
%   Author: Wenxiang Wu
%   Date:   2025-02-20

    xiPast=xiInputs.statesPast;
    xiNow=xiInputs.statesNow;
    K_omega=ship.observer.K_omega;
    dynamics=ship.dynamics;
    m11=dynamics.m11; m22=dynamics.m22; m23=dynamics.m23; m32=dynamics.m32; m33=dynamics.m33; Xu=dynamics.Xu; Xuu=dynamics.Xuu; Xuuu=dynamics.Xuuu;Yv=dynamics.Yv; Yvv=dynamics.Yvv;
    Yr=dynamics.Yr; Yrv=dynamics.Yrv; Yvr=dynamics.Yvr; Yrr=dynamics.Yrr; Nv=dynamics.Nv; Nvv=dynamics.Nvv; Nr=dynamics.Nr; Nrv=dynamics.Nrv; Nvr=dynamics.Nvr; Nrr=dynamics.Nrr;%水动力参数   
    M=[
        m11  0    0
        0   m22  m23
        0   m32  m33
        ];
    Belta=observer.Belta';
    U(1,1)=xiPast(1,4);
    U(2,1)=xiPast(1,5);
    U(3,1)=xiPast(1,6);
    ui=commands';
    
    C_U=[
        0                      0      -m22*U(2,1)-m23*U(3,1)
        0                      0           m11*U(1,1)
        m22*U(2,1)+m23*U(3,1)  -m11*U(1,1)          0
        ];

    D_U=[
        -Xu-Xuu*abs(U(1,1))-Xuuu*U(1,1)^2 0 0
        0 -Yv-Yvv*abs(U(2,1))-Yrv*abs(U(3,1)) -Yr-Yvr*abs(U(2,1))-Yrr*abs(U(3,1))
        0 -Nv-Nvv*abs(U(2,1))-Nrv*abs(U(3,1)) -Nr-Nvr*abs(U(2,1))-Nrr*abs(U(3,1))
        ];
    BeltaDot = -K_omega*Belta-K_omega*(-C_U*U-D_U*U+ui+K_omega*M*U); 
    Belta=Belta+BeltaDot*Ts;
    observer.UnDis=(Belta+K_omega*M*xiNow(1,4:6)')';
    observer.Belta=Belta';   
    
end

