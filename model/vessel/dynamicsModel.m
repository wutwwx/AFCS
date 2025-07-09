function xdot = dynamicsModel(ship,xi,ui,disturbance) 
%DYNAMICSMODEL  Computes state derivative for Fossen/MMG surface ship models.
%
%   xdot = dynamicsModel(ship, xi, ui, disturbance)
%   Calculates the state derivative vector for the specified ship model (Fossen type or MMG type),
%   including external disturbances (current, wind, unknown force).
%
%   Inputs:
%     ship        - Struct. Ship parameters (model type, hydrodynamics, geometry, etc.)
%     xi          - State vector (length depends on ship.type)
%                   Fossen: [x, y, psi, u, v, r]
%                   MMG:    [x, y, psi, u, v, r, delta, n]
%     ui          - Control input vector (rudder/propeller or tau)
%     disturbance - Struct with fields:
%                    .current   - 3×1 vector, earth-fixed [x; y; psi] currents (m/s or rad/s)
%                    .force     - 3×1 vector, [X, Y, N] unknown external forces/moments (N, N, Nm)
%
%   Output:
%     xdot - State derivative vector (same length as xi)
%
%   Model types:
%     ship.type==1  : 6-DOF Fossen model (tau-driven, see [Fossen, 2011])
%     ship.type==2  : MMG model (rudder-propeller-driven, see "Introduction of MMG standard method for ship maneuvering predictions")
%
%   Reference:
%     [1] Fossen, T. I., "Guidance and Control of Ocean Vehicles", Wiley, 1994.
%     [2] Yasukawa, H., & Yoshimura, Y. Introduction of MMG standard method for ship maneuvering predictions.
%        Journal of Marine Science and Technology, 2015 Mar;20:37-52.
%
%   Author: Wenxiang Wu
%   Date:   2025-03-19

    current=disturbance.current';
    d_w=disturbance.force';
if ship.type==1
    m11=ship.m11; m22=ship.m22; m23=ship.m23; m32=ship.m32; m33=ship.m33; Xu=ship.Xu; Xuu=ship.Xuu; Xuuu=ship.Xuuu;Yv=ship.Yv; Yvv=ship.Yvv;
    Yr=ship.Yr; Yrv=ship.Yrv; Yvr=ship.Yvr; Yrr=ship.Yrr; Nv=ship.Nv; Nvv=ship.Nvv; Nr=ship.Nr; Nrv=ship.Nrv; Nvr=ship.Nvr; Nrr=ship.Nrr;%水动力参数
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

    xdot_1=R*U+current;
    xdot_2=M^(-1)*(ui+d_w-C_U*U-D_U*U);
    xdot=[xdot_1;xdot_2];
elseif ship.type==2
    U = sqrt(xi(4)^2 + xi(5)^2);
    if U < 0.01
        U = 0.01; % Prevent division by zero
    end
    rho = ship.rho; L = ship.L; d = ship.d; delta_max = ship.delta_max; n_max = ship.n_max;
    delta_c = ui(1); n_c = ui(2) * L / U;
    u = xi(4)/U; v = xi(5)/U; r = xi(6)*L/U; psi = xi(3); delta = xi(7); n = xi(8)*L/U;
    if u == 0
       u = 1e-5;
    end
    if abs(n) < 1
       n = 1;
    end
    m   = ship.m; mx  = ship.mx; my  = ship.my; Jzz = ship.Jzz; Izz = ship.Izz;
    % CFD coefficients
    Xuu  = ship.Xuu;  Xvv  = ship.Xvv;  Xvr  = ship.Xvr;  Xrr  = ship.Xrr;  Yv = ship.Yv; Yr = ship.Yr;
    Yvvv = ship.Yvvv; Yvvr = ship.Yvvr; Yvrr = ship.Yvrr; Yrrr = ship.Yrrr; Nv = ship.Nv; Nr = ship.Nr;
    Nvvv = ship.Nvvv; Nvvr = ship.Nvvr; Nvrr = ship.Nvrr; Nrrr = ship.Nrrr;
    % Propeller and rudder coefficients
    tR = ship.tR; aH = ship.aH; xH = ship.xH; xR = ship.xR; AR = ship.AR; Delta = ship.Delta; kk = ship.kk;
    uP = ship.uP; tP = ship.tP; gammaR1 = ship.gammaR1; gammaR2 = ship.gammaR2; DP = ship.DP; lR = ship.lR;
    eta = ship.eta; epsilon = ship.epsilon;
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
    KT = ship.KT; J1 = ship.J1;
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
    error('Error: This dynamics model type is not existed');
end
 
end
    
