function Winds = windsSet(ASVs, winds, nowStates,  modelType , t)
%WINDSSET   Calculate wind disturbance forces for ASV(s), supporting Isherwood (1972) and Blendermann (1994).
%
%   Winds = windsSet(ASVs, winds, nowStates, t, modelType)
%   Inputs:
%     ASVs      : Cell array of ASV parameter structures (with .dynamics field)
%     winds      : Structure with fields:
%                   - speed : wind speed (m/s) (optional; default = 2)
%                   - angle : wind direction, radians (optional; default = pi/3)
%     nowStates  : [n_ASVs × state_dim] matrix, each row is the state vector of a ASV
%     t          : Current simulation time (s)
%     modelType  : (optional) 'isherwood' (default) or 'blendermann'
%
%   Outputs:
%     Winds      : Cell array, each cell is a struct with fields:
%                   - forces : [F_u, F_v, F_r] (N, N, Nm), wind force/moment
%                   - speed  : wind speed used (m/s)
%                   - angle  : wind direction used (rad)
%
%   References:
%     [1] Isherwood RM, "Wind Resistance of Merchant ASVs." Trans. RINA. 1973;115:327-38.
%     [2] Blendermann W, "Parameter identification of wind loads on ASVs."
%     Journal of Wind Engineering and Industrial Aerodynamics. 1994 May 1;51(3):339-51.

%
%   Example:
%     Winds = windsSet(ASVs, winds, nowStates, t, 'blendermann');
%
%   Author: Wenxiang Wu (with ChatGPT enhancement)
%   Date:   2025-03-20

if nargin < 5 || isempty(modelType)
    modelType = 'isherwood';
end

if isempty(winds)
    windspeed = 2;
    angle_w = pi / 3;
else
    windspeed = winds.speed;
    angle_w = winds.angle;
end

Winds = cell(1, length(ASVs));
rho_a = 1.225;

uwindN = windspeed * cos(angle_w + pi);
uwindE = windspeed * sin(angle_w + pi);

for j = 1:length(ASVs)
    psi = nowStates(j,3);
    u_ASV = nowStates(j,4);
    v_ASV = nowStates(j,5);
    UN = cos(psi)*u_ASV - sin(psi)*v_ASV;
    UE = sin(psi)*u_ASV + cos(psi)*v_ASV;
    V_r = sqrt((uwindN - UN)^2 + (uwindE - UE)^2);

    gamma_r = mod(psi - angle_w, 2*pi);
    if gamma_r > pi, gamma_r = gamma_r - 2*pi; end

    ASVDyn = ASVs{j}.dynamics;

    switch lower(modelType)
        case 'blendermann'
            % Blendermann (1994) wind load model
            vessel_no = getfield_safe(ASVDyn, 'vessel_no', 2);
            ALw = getfield_safe(ASVDyn, 'A_s', 6);
            AFw = getfield_safe(ASVDyn, 'A_f', 2);
            sH  = getfield_safe(ASVDyn, 'sH', 0.3);
            sL  = getfield_safe(ASVDyn, 'sL', 0.5);
            Loa = getfield_safe(ASVDyn, 'L', 5);
            [tau_w, ~, ~, ~, ~] = blendermann94(gamma_r, V_r, AFw, ALw, sH, sL, Loa, vessel_no);
            F_u = tau_w(1); F_v = tau_w(2); F_r = tau_w(3);

        case 'isherwood'
            % Isherwood (1972) wind load model
            Loa  = getfield_safe(ASVDyn, 'L', 5);
            B    = getfield_safe(ASVDyn, 'B', 1);
            ALw  = getfield_safe(ASVDyn, 'A_s', 6);
            AFw  = getfield_safe(ASVDyn, 'A_f', 2);
            A_SS = getfield_safe(ASVDyn, 'A_ss', 1);
            S    = getfield_safe(ASVDyn, 'C', 2);
            Cval = getfield_safe(ASVDyn, 'e', 1);
            M    = getfield_safe(ASVDyn, 'Mn', 0);
            [tau_w, ~, ~, ~] = isherwood72(gamma_r, V_r, Loa, B, ALw, AFw, A_SS, S, Cval, M);
            F_u = tau_w(1); F_v = tau_w(2); F_r = tau_w(3);

        otherwise
            error('Unknown wind model type: %s. Use "isherwood" or "blendermann".', modelType);
    end

    Winds{j}.forces = [F_u, F_v, F_r];
    Winds{j}.speed  = windspeed;
    Winds{j}.angle  = angle_w;
end
end

function v = getfield_safe(S, fname, default)
    if isfield(S, fname)
        v = S.(fname);
    else
        v = default;
    end
end

function [tau_w, CX, CY, CN] = isherwood72(gamma_r, V_r, Loa, B, ALw, AFw, A_SS, S, C, M)
    rho_a = 1.224;
    gamma_r_deg = mod(rad2deg(gamma_r),360);
    if gamma_r_deg > 180
        gamma_r_deg = 360 - gamma_r_deg;
    end
    CX_data = [...  
    0	2.152	-5.00	0.243	-0.164	0	0	0	
    10	1.714	-3.33	0.145	-0.121	0	0	0	
    20	1.818	-3.97	0.211	-0.143	0	0	0.033	
    30	1.965	-4.81	0.243	-0.154	0	0	0.041	
    40	2.333	-5.99	0.247	-0.190	0	0	0.042
    50	1.726	-6.54	0.189	-0.173	0.348	0	0.048	
    60	0.913	-4.68	0	-0.104	0.482	0	0.052	
    70	0.457	-2.88	0	-0.068	0.346	0	0.043	
    80	0.341	-0.91	0	-0.031	0	0	0.032	
    90	0.355	0	0	0	-0.247	0	0.018	
    100	0.601	0	0	0	-0.372	0	-0.020
    110	0.651	1.29	0	0	-0.582	0	-0.031	
    120	0.564	2.54	0	0	-0.748	0	-0.024	
    130	-0.142	3.58	0	0.047	-0.700	0	-0.028	
    140	-0.677	3.64	0	0.069	-0.529	0	-0.032	
    150	-0.723	3.14	0	0.064	-0.475	0	-0.032	
    160	-2.148	2.56	0	0.081	0	1.27	-0.027	
    170	-2.707	3.97	-0.175	0.126	0	1.81	0	
    180	-2.529	3.76	-0.174	0.128	0	1.55	0];
    CY_data = [
    0	0	0	0	0	0	0	0
    10	0.096	0.22	0	0	0	0	0
    20	0.176	0.71	0	0	0	0	0
    30	0.225	1.38	0	0.023	0	-0.29	0
    40	0.329	1.82	0	0.043	0	-0.59	0
    50	1.164	1.26	0.121	0	-0.242	-0.95	0
    60	1.163	0.96	0.101	0	-0.177	-0.88	0
    70	0.916	0.53	0.069	0	0	-0.65	0
    80	0.844	0.55	0.082	0	0	-0.54	0
    90	0.889	0	0.138	0	0	-0.66	0
    100	0.799	0	0.155	0	0	-0.55	0
    110	0.797	0	0.151	0	0	-0.55	0
    120	0.996	0	0.184	0	-0.212	-0.66	0.34
    130	1.014	0	0.191	0	-0.280	-0.69	0.44
    140	0.784	0	0.166	0	-0.209	-0.53	0.38
    150	0.536	0	0.176	-0.029	-0.163	0	0.27
    160	0.251	0	0.106	-0.022	0	0	0
    170	0.125	0	0.046	-0.012	0	0	0
    180	0	0	0	0	0	0	0];
    CN_data = [
    0	0	0	0	0	0	0
    10	0.0596	0.061	0	0	0	-0.074
    20	0.1106	0.204	0	0	0	-0.170
    30	0.2258	0.245	0	0	0	-0.380
    40	0.2017	0.457	0	0.0067	0	-0.472
    50	0.1759	0.573	0	0.0118	0	-0.523
    60	0.1925	0.480	0	0.0115	0	-0.546
    70	0.2133	0.315	0	0.0081	0	-0.526
    80	0.1827	0.254	0	0.0053	0	-0.443
    90	0.2627	0	0	0	0	-0.508
    100	0.2102	0	-0.0195	0	0.0335	-0.492
    110	0.1567	0	-0.0258	0	0.0497	-0.457
    120	0.0801	0	-0.0311	0	0.0740	-0.396
    130	-0.0189	0	-0.0488	0.0101	0.1128	-0.420
    140	0.0256	0	-0.0422	0.0100	0.0889	-0.463
    150	0.0552	0	-0.0381	0.0109	0.0689	-0.476
    160	0.0881	0	-0.0306	0.0091	0.0366	-0.415
    170	0.0851	0	-0.0122	0.0025	0	-0.220
    180	0	0	0	0	0	0	0];

    A0 = interp1(CX_data(:,1),CX_data(:,2),gamma_r_deg);
    A1 = interp1(CX_data(:,1),CX_data(:,3),gamma_r_deg);
    A2 = interp1(CX_data(:,1),CX_data(:,4),gamma_r_deg);
    A3 = interp1(CX_data(:,1),CX_data(:,5),gamma_r_deg);
    A4 = interp1(CX_data(:,1),CX_data(:,6),gamma_r_deg);
    A5 = interp1(CX_data(:,1),CX_data(:,7),gamma_r_deg);
    A6 = interp1(CX_data(:,1),CX_data(:,8),gamma_r_deg);

    B0 = interp1(CY_data(:,1),CY_data(:,2),gamma_r_deg);
    B1 = interp1(CY_data(:,1),CY_data(:,3),gamma_r_deg);
    B2 = interp1(CY_data(:,1),CY_data(:,4),gamma_r_deg);
    B3 = interp1(CY_data(:,1),CY_data(:,5),gamma_r_deg);
    B4 = interp1(CY_data(:,1),CY_data(:,6),gamma_r_deg);
    B5 = interp1(CY_data(:,1),CY_data(:,7),gamma_r_deg);
    B6 = interp1(CY_data(:,1),CY_data(:,8),gamma_r_deg);

    C0 = interp1(CN_data(:,1),CN_data(:,2),gamma_r_deg);
    C1 = interp1(CN_data(:,1),CN_data(:,3),gamma_r_deg);
    C2 = interp1(CN_data(:,1),CN_data(:,4),gamma_r_deg);
    C3 = interp1(CN_data(:,1),CN_data(:,5),gamma_r_deg);
    C4 = interp1(CN_data(:,1),CN_data(:,6),gamma_r_deg);
    C5 = interp1(CN_data(:,1),CN_data(:,7),gamma_r_deg);

    CX =  -(A0 + A1*2*ALw/Loa^2 + A2*2*AFw/B^2 + A3*(Loa/B) + ...
            A4*(S/Loa) + A5*(C/Loa) + A6*M);
    CY =    B0 + B1*2*ALw/Loa^2 + B2*2*AFw/B^2 + B3*(Loa/B) + ...
            B4*(S/Loa) + B5*(C/Loa) + B6*A_SS/ALw;
    CN =    C0 + C1*2*ALw/Loa^2 + C2*2*AFw/B^2 + C3*(Loa/B) + ...
            C4*(S/Loa) + C5*(C/Loa);

    tauX = 0.5 * CX * rho_a * V_r^2 * AFw;
    tauY = 0.5 * CY * rho_a * V_r^2 * ALw .* sign(gamma_r);
    tauN = 0.5 * CN * rho_a * V_r^2 * ALw * Loa .* sign(gamma_r);

    tau_w = [tauX, tauY, tauN]';
end

function [tau_w,CX,CY,CK,CN] = blendermann94(gamma_r,V_r,AFw,ALw,sH,sL,Loa,vessel_no)
    rho_a = 1.224;
    BDATA = [
        0.95 0.55 0.60 0.80 1.2
        0.85 0.65 0.55 0.40 1.7
        0.85 0.55 0.50 0.40 1.4
        0.90 0.55 0.55 0.40 1.4
        0.85 0.60 0.65 0.65 1.1
        0.90 0.60 0.80 0.55 1.7
        1.00 0.85 0.925 0.10 1.7
        0.90 0.45 0.50 0.80 1.1
        0.95 0.70 0.70 0.40 1.1
        0.70 0.60 0.65 0.50 1.1
        0.90 0.55 0.80 0.55 1.2
        0.90 0.40 0.40 0.80 1.2
        0.85 0.55 0.65 0.60 1.4
        0.90 0.55 0.60 0.60 1.1
        0.70 0.90 0.55 0.40 3.1
        0.70 0.75 0.55 0.40 2.2
        0.85 0.55 0.55 0.65 1.1];
    vessel_no = min(max(1, round(vessel_no)), size(BDATA,1));
    CDt  = BDATA(vessel_no,1);
    CDl_AF_bow = BDATA(vessel_no,2);
    CDl_AF_stern = BDATA(vessel_no,3);
    delta = BDATA(vessel_no,4);
    kappa = BDATA(vessel_no,5);

    gamma_r = mod(gamma_r, 2*pi);
    for i = 1:length(gamma_r)
        if abs(gamma_r(i)) <= pi/2
            CDlAF(i,1) = CDl_AF_bow;
        else
            CDlAF(i,1) = CDl_AF_stern;
        end
    end
    CDl = CDlAF * AFw / ALw;
    den = 1 - 0.5 * delta * (1-CDl/CDt) .* sin(2 * gamma_r).^2;
    CX = -CDlAF .* cos(gamma_r) ./ den;
    CY =  CDt * sin(gamma_r) ./ den;
    CN =  ( sL/Loa - 0.18 * (gamma_r - pi/2) ) .* CY;

    tauX = 0.5 * CX * rho_a * V_r^2 * AFw;
    tauY = 0.5 * CY * rho_a * V_r^2 * ALw;
    tauN = 0.5 * CN * rho_a * V_r^2 * ALw * Loa;
    tau_w = [tauX, tauY, tauN]';
    CK = kappa;
end
