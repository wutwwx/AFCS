function xdot = PreDynamicsModel(ASV,xi,ui,disturbance) 
%PREDYNAMICSMODEL   ASV state propagation (3DOF/underactuated) for prediction/control.
%
%   xdot = PreDynamicsModel(ASV, xi, ui, disturbance)
%   Computes the time derivative of ASV state for use in prediction (e.g., in MPC),
%   supporting both Fossen 3-DOF model (type==1) and MMG (Manoeuvring Mathematical Modelling Group) model (type==2).
%
%   Inputs:
%     ASV        - Struct, ASV dynamic parameters (fields depend on .type)
%     xi          - State vector at current step (6×1 for Fossen, 8×1 for MMG)
%     ui          - Control input vector at current step
%                   [tau_u, tau_v, tau_r] for Fossen; [delta_c, n_c] for MMG
%     disturbance - Struct, .current (1×3), .force (1×3)
%
%   Output:
%     xdot        - State derivative vector (same dimension as xi)
%
%   Example usage:
%     xdot = PreDynamicsModel(ASV, xi, ui, disturbance)
%
%   Reference:
%     [1] Fossen, T. I., "Guidance and Control of Ocean Vehicles", Wiley, 1994.
%     [2] Yasukawa, H., & Yoshimura, Y. Introduction of MMG standard method for ASV maneuvering predictions.
%        Journal of Marine Science and Technology, 2015 Mar;20:37-52.

%   Author: Wenxiang Wu
%   Date:   2025-04-15

    c_v=disturbance.current';
    d_w=disturbance.force';
if ASV.type==1    
    m11=ASV.m11; m22=ASV.m22; m23=ASV.m23; m32=ASV.m32; m33=ASV.m33; Xu=ASV.Xu; Xuu=ASV.Xuu; Xuuu=ASV.Xuuu;Yv=ASV.Yv; Yvv=ASV.Yvv;
    Yr=ASV.Yr; Yrv=ASV.Yrv; Yvr=ASV.Yvr; Yrr=ASV.Yrr; Nv=ASV.Nv; Nvv=ASV.Nvv; Nr=ASV.Nr; Nrv=ASV.Nrv; Nvr=ASV.Nvr; Nrr=ASV.Nrr;%水动力参数
    phi = xi(3);  % phi
    U(1,1) = xi(4);
    U(2,1) = xi(5);
    U(3,1) = xi(6);


    M=[
        m11  0    0
        0   m22  m23
        0   m32  m33
        ];

    R=[
        cos(phi) -sin(phi) 0
        sin(phi) cos(phi)  0
            0        0     1
        ];

    C_U=[
        0                      0      -m22*U(2)-m23*U(3)
        0                      0      m11*U(1)
        m22*U(2)+m23*U(3)  -m11*U(1)          0
        ];

    D_U=[
        -Xu-Xuu*abs(U(1))-Xuuu*U(1)^2 0 0
        0 -Yv-Yvv*abs(U(2))-Yrv*abs(U(3)) -Yr-Yvr*abs(U(2))-Yrr*abs(U(3))
        0 -Nv-Nvv*abs(U(2))-Nrv*abs(U(3)) -Nr-Nvr*abs(U(2))-Nrr*abs(U(3))
        ];

    xdot_1=R*U+c_v;
    xdot_2=M^(-1)*(ui-C_U*U-D_U*U+d_w);
    xdot=[xdot_1;xdot_2];
elseif ASV.type==2
    U = sqrt(xi(4)^2 + xi(5)^2);
    if U < 0.01
        U = 0.01; % Prevent division by zero
    end
    rho = ASV.rho; L = ASV.L; d = ASV.d; delta_max = ASV.delta_max; n_max = ASV.n_max;
    delta_c = ui(1); n_c = ui(2) * L / U;
    u = xi(4)/U; v = xi(5)/U; r = xi(6)*L/U; psi = xi(3); delta = xi(7); n = xi(8)*L/U;
    if u == 0
       u = 1e-5;
    end
    if abs(n) < 1
       n = 1;
    end
    m   = ASV.m; mx  = ASV.mx; my  = ASV.my; Jzz = ASV.Jzz; Izz = ASV.Izz;
    % CFD coefficients
    Xuu  = ASV.Xuu;  Xvv  = ASV.Xvv;  Xvr  = ASV.Xvr;  Xrr  = ASV.Xrr;  Yv = ASV.Yv; Yr = ASV.Yr;
    Yvvv = ASV.Yvvv; Yvvr = ASV.Yvvr; Yvrr = ASV.Yvrr; Yrrr = ASV.Yrrr; Nv = ASV.Nv; Nr = ASV.Nr;
    Nvvv = ASV.Nvvv; Nvvr = ASV.Nvvr; Nvrr = ASV.Nvrr; Nrrr = ASV.Nrrr;
    % Propeller and rudder coefficients
    tR = ASV.tR; aH = ASV.aH; xH = ASV.xH; xR = ASV.xR; AR = ASV.AR; Delta = ASV.Delta; kk = ASV.kk;
    uP = ASV.uP; tP = ASV.tP; gammaR1 = ASV.gammaR1; gammaR2 = ASV.gammaR2; DP = ASV.DP; lR = ASV.lR;
    eta = ASV.eta; epsilon = ASV.epsilon;
    if abs(delta_c) >= delta_max
        delta_c = sign(delta_c) * delta_max;
    end
    if delta < 0
        gammaR = gammaR2;
    else
        gammaR = gammaR1;
    end
    if delta_c ~= delta
        delta_dot = abs(delta_c - delta) * sign(delta_c - delta);
    else
        delta_dot = 0;
    end
    n_c = n_c * U / L;
    n = n * U / L;
    if abs(n_c) >= n_max/60
        n_c = sign(n_c) * n_max/60;
    end
    KT = ASV.KT; J1 = ASV.J1;
    n_dot = (n_c - n) ; 
    J2 = uP * U / (n * DP);
    if J1 ~= J2
        KT = ((2 * J1 - J2) / J1)^1 * KT;
    end   
    uR = uP * epsilon * u * U * sqrt(eta * (1 + kk * (sqrt(1 + 8 * 0.28 / (pi * 0.3^2)) - 1))^2 + 1 - eta);
    vR = gammaR * (v * U - r * U / L * lR);
    fR = 6.13 * Delta / (2.25 + Delta);
    alphaR = delta + atan(vR/uR);
    FN = -AR/(L*d*U^2)*fR*(uR^2 + vR^2)*sin(alphaR);
    % Hull
    X_H = Xuu * u^2 + Xvv * v^2 + Xvr * v * r + Xrr * r^2;
    Y_H = Yv * v + Yr * r + Yvvr * v^2 * r + Yvrr * v * r^2 + Yvvv * v^3 + Yrrr * r^3;
    N_H = Nv * v + Nr * r + Nvvr * v^2 * r + Nvrr * v * r^2 + Nrrr * r^3 + Nvvv * v^3;
    % Rudder
    X_R = (1 - tR) * FN * sin(delta);
    Y_R = (1 + aH) * FN * cos(delta);
    N_R = (xR + aH * xH) * FN * cos(delta);
    % Propeller
    X_P = rho * DP^4 * n^2 * (1 - tP) * KT / (0.5 * U^2 * L * d * rho);
    % Disturbance force   
    X_dist = d_w(1,1) / (0.5 * rho * U^2 * L * d);
    Y_dist = d_w(2,1) / (0.5 * rho * U^2 * L * d);
    N_dist = d_w(3,1) / (0.5 * rho * U^2 * L^2 * d);    
    
    
    X = X_H + X_R + X_P + X_dist;
    Y = Y_H + Y_R + Y_dist;
    N = N_H  + N_R + N_dist;

    
     xdot = [(u * cos(psi) - v * sin(psi)) * U + c_v(1,1)
        (u * sin(psi) + v * cos(psi)) * U + c_v(2,1)
        r * (U / L) 
         (X + (m + my) * v * r) / (m + mx) * (U^2 / L)
        (Y - (m + mx) * u * r) / (m + my) * (U^2 / L)
        (N / (Jzz + Izz) * (U^2 / L^2))        
        delta_dot
        n_dot
    ];


else
    error('Error: This prediction dynamics model type is not existed');
end

