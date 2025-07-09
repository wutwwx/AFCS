function Waves = wavesSet(ships,waves,nowStates,t)
%WAVESSET   Compute wave-induced drift forces and moments for ship(s).
%
%   Waves = wavesSet(ships, waves, nowStates, t) calculates the wave drift 
%   forces (surge, sway, yaw) acting on one or more ships using either a 
%   regular or irregular wave model, based on ship parameters, present state, 
%   and environmental wave conditions.
%
%   Inputs:
%     ships      : Cell array of ship parameter structures (with .dynamics field)
%     waves      : Structure with fields:
%                   - type    : 'regular' or 'irregular'
%                   - angle   : Wave propagation direction (rad)
%                   - CounterM: Number of frequency components (irregular waves)
%                   - rho     : Seawater density (kg/m^3)
%                   - Vwimd   : Wind speed (m/s) [optional, used in Pierson-Moskowitz spectrum]
%     nowStates  : [n_ships × state_dim] matrix, each row is the state vector of a ship
%                  (expected: [x, y, psi, u, v, r] or similar order)
%     t          : Current simulation time (s)
%
%   Outputs:
%     Waves      : Cell array, each cell is a struct with fields:
%                   - force     : [Xwave, Ywave, Nwave] (N, N, Nm), total drift force and yaw moment
%                   - angle     : Wave propagation direction used (rad)
%                   - frequency : Main wave frequency used (regular waves only)
%
%   Model summary:
%     - For 'regular' waves, computes drift force using a single dominant wave frequency.
%     - For 'irregular' waves, integrates drift effects from multiple frequency components 
%       sampled from the Pierson-Moskowitz spectrum (random phase).
%     - Empirical coefficients (Cxw, Cyw, Cnw) are calculated as per standard experimental data.
%
%   Reference:
%     - Fossen, T. I. (2011). "Handbook of Marine Craft Hydrodynamics and Motion Control," Wiley, Sec. 9.7.
%     - Pierson, W. J. & Moskowitz, L. (1964). "A proposed spectral form for fully developed wind seas..."
%
%   Author: Wangchen Guang
%   Date:   2025-02-25
%
%   Example usage:
%     Waves = wavesSet(ships, waves, nowStates, t);
%
if isempty(waves)
    waves_type='regular';
    Chi=0;                                        % Wave propagation direction
    CounterM=20;                                  % Number of discrete wave frequencies
    rho=1025.0;                                   % Seawater density
    V_wind=0;                                     % Wind speed (m/s)
else
    waves_type=waves.type;
    Chi=waves.angle;                              % Wave propagation direction
    CounterM=waves.CounterM;                      % Number of discrete wave frequencies
    rho=waves.rho;                                % Seawater density
    V_wind=waves.Vwimd;                           % Wind speed (m/s)
end


Waves=cell(1,length(ships));


g=9.81;                                           % Gravitational acceleration  


A=8.1e-3*g^2;                                     % Pierson-Moskowitz spectrum  
B=0.74*(g/(V_wind+10^(-9)))^4;
m0=A/(4*B);                                       % Spectral moment
meanh=4*sqrt(m0);
miu=0.002;
wmin=(-3.11/(meanh^2*log(miu)))^(1/4);            % Minimum wave frequency
wmax=(-3.11/(meanh^2*log(1-miu)))^(1/4);
delta_w=(wmax-wmin)/CounterM;

if strcmp(waves_type, 'regular')
    for j=1:length(ships)
        L=ships{j}.dynamics.L; 
        psi=nowStates(j,3);
        % Regular wave: use a single frequency (e.g., midpoint frequency)
        w = (wmin + wmax) / 2+10^(-9);            % Regular wave frequency
        a = sqrt(2 * A * exp(-B / w^4) / w^5);    % Wave amplitude
        lamda_w = 2 * pi * w^2 / g;               % Deep water wavelength

        % Experimental coefficients
        Cxw = 0.05 - 0.2 * (lamda_w / L) + 0.75 * (lamda_w / L)^2 - 0.51 * (lamda_w / L)^3;
        Cyw = 0.46 + 6.83 * (lamda_w / L) - 15.65 * (lamda_w / L)^2 + 8.44 * (lamda_w / L)^3;
        Cnw = -0.11 + 0.68 * (lamda_w / L) - 0.79 * (lamda_w / L)^2 + 0.21 * (lamda_w / L)^3;

        % Wave drift force and moment from regular wave
        Xwave = 0.5 * rho * L * a^2 * cos(pi-Chi+psi) * Cxw;
        Ywave = 0.5 * rho * L * a^2 * sin(pi-Chi+psi) * Cyw;
        Nwave = 0.5 * rho * L * a^2 * sin(pi-Chi+psi) * Cnw;
        Waves{j}.force = [Xwave, Ywave, Nwave];
        Waves{j}.angle = Chi;
        Waves{j}.frequency = w;
    end
elseif strcmp(waves_type, 'irregular')
    for j=1:length(ships)
        Xwave=0;Ywave=0;Nwave=0;  
        L=ships{j}.dynamics.L;                   % Ship length
        psi=nowStates(j,3);                      % Heading angle
        % Wave drift forces and moments in irregular waves: superposition of multiple regular wave components with random frequencies
        for i=1:CounterM
            jp=fix(1+(CounterM-1)*rand(1));       % Generate random integer in [1, CounterM]
            w=wmin+(jp-1)*delta_w;                % Discrete wave frequency
            S=A*exp(-B/w^4)/w^5;                  % Spectral density value
            lamda_w=2*pi*w^2/g;                   % Consider deep water wave
            Cxw= 0.05-0.2*(lamda_w/L) +0.75*(lamda_w/L)^2 -0.51*(lamda_w/L)^3;  % Experimental coefficient
            Cyw= 0.46+6.83*(lamda_w/L)-15.65*(lamda_w/L)^2+8.44*(lamda_w/L)^3;
            Cnw=-0.11+0.68*(lamda_w/L)-0.79*(lamda_w/L)^2 +0.21*(lamda_w/L)^3;
            Xwave=Xwave+rho*L*cos(pi-Chi+psi)*Cxw*S*delta_w;        
            Ywave=Ywave+0.5*rho*L*sin(pi-Chi+psi)*Cyw*S*delta_w;
            Nwave=Nwave+0.5*rho*L^2*sin(pi-Chi+psi)*Cnw*S*delta_w;
        end
    Waves{j}.force = [Xwave, Ywave, Nwave];
    Waves{j}.angle = Chi;
    end
else
    error('Invalid waves type. Choose "regular" or "irregular".');
end
end


