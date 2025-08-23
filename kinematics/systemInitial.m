function  System= systemInitial(N,ASVs,env,conditions) 
%SYSTEMINITIAL   Initialize multi-ASV simulation system states and environment conditions.
%
%   System = systemInitial(N, ASVs, env, conditions)
%   Initializes the state trajectory, command arrays, and all auxiliary states
%   (such as disturbance, noise, and fault variables) for a given fleet and
%   environmental scenario, including various uncertainties and fault modes.
%
%   Inputs:
%     N           - Integer, total simulation steps
%     ASVs       - Cell array, each ASV's parameter struct (fields: .dynamics, .observer, .controller, etc.)
%     env         - Struct, environmental parameter fields (e.g., .winds, .waves, .currents, ...)
%     conditions  - String/cell array, enabled environment/fault conditions (see below)
%
%   Supported conditions:
%     "winds"               - Enable wind disturbance
%     "waves"               - Enable wave disturbance
%     "currents"            - Enable current disturbance
%     "measureNoise"        - Add measurement noise
%     "sensorFault"         - Enable sensor faults
%     "actuatorFault"       - Enable actuator faults
%     "communicationFault"  - Enable inter-ASV communication faults
%     "unknownDisturbance"  - Add unmodeled external disturbance (for robust/observer test)
%
%   Output:
%     System      - Cell array, each ASV's state struct with initialized arrays
%
%   Example usage:
%     System = systemInitial(N, ASVs, env, {"winds", "currents", "unknownDisturbance"});
%
%   Note:
%     - Cybership2 initial states and Yunfan1 (MMG) initial states are provided; uncomment as appropriate.
%     - All field sizes are preallocated for vectorized simulation.
%
%   Author: Wenxiang Wu
%   Date:   2025-05-20

ASVNum=length(ASVs);
System = cell(1, ASVNum);
for j=1:ASVNum
    Nx(1,j)=ASVs{j}.dynamics.Nx;
    Nu(1,j)=ASVs{j}.dynamics.Nu;
end
for j=1:ASVNum
    System{j}.realStates=zeros(N,Nx(1,j));
    System{j}.commands=zeros(N,Nu(1,j));   
end

% Cybership2
System{1}.realStates(1,:)=[4,15,0,0,0,0]; % x y psi u v r 
System{2}.realStates(1,:)=[0.2,11.4,0,0,0,0];   
System{3}.realStates(1,:)=[0.2,15,0,0,0,0]; 
System{4}.realStates(1,:)=[-4,18.6,0,0,0,0];  

% Yunfan1
% System{1}.realStates(1,:)=[4,15,0,0,0,0,0,1000/60]; % x y psi u v r delta n
% System{2}.realStates(1,:)=[0.2,11.4,0,0,0,0,0,1000/60];   
% System{3}.realStates(1,:)=[0.2,15,0,0,0,0,0,1000/60]; 
% System{4}.realStates(1,:)=[-4,18.6,0,0,0,0,0,1000/60];  


if length(System)~=ASVNum
    error('Error: The setting initial states does not match the number of ASVs ');
end
for j=1:ASVNum
    if isfield(ASVs{j},'observer')
        System{j}.obserStates=[];
    end
    System{j}.controlStates=[];
end

if ~isempty(conditions)
    allowed_strings = ["winds", "waves", "currents", "measureNoise", "sensorFault", "actuatorFault", "communicationFault","unknownDisturbance"];  
    invalid_conditions = conditions(~ismember(conditions, allowed_strings));
    if ~isempty(invalid_conditions)
        error('Error: The following conditions are not allowed: %s', strjoin(invalid_conditions, ', '));
    end

    for j=1:ASVNum
    if length(System{j}.realStates(1,:))~=ASVs{j}.dynamics.Nx
        error(['The dimension of the initialized state of ' num2str(j) 'th ASV is different from the its model state dimension']);
    end
    end

    realStatesNow=System{1}.realStates(1,:);
    commandsNow=System{1}.commands(1,:);
    for j=2:ASVNum
        realStatesNow(end+1, :)=System{j}.realStates(1,:);
        commandsNow(end+1, :)=System{j}.commands(1,:);
    end

    if ismember("unknownDisturbance", conditions)==true
        UnDis=unknownDisturbance(ASVNum,realStatesNow,0);
        for j=1:ASVNum
            System{j}.UnDis=zeros(N,3);
            System{j}.UnDis(1,:)=UnDis{j};
        end
    end

    if ismember("winds", conditions)==true
        winds=windsSet(ASVs,env.winds,realStatesNow,'blendermann',0);
        for j=1:ASVNum
            System{j}.winds.force=zeros(N,3);
            System{j}.winds.speed=zeros(N,1);
            System{j}.winds.angle=zeros(N,1);
            System{j}.winds.force(1,:)=winds{j}.forces;
            System{j}.winds.speed(1,:)=winds{j}.speed;
            System{j}.winds.angle(1,:)=winds{j}.angle;
        end
    end

    if ismember("waves", conditions)==true
        waves=wavesSet(ASVs,env.waves,realStatesNow,0);
        for j=1:ASVNum
            System{j}.waves.force=zeros(N,3);
            System{j}.waves.angle=zeros(N,1);
            System{j}.waves.force(1,:)=waves{j}.force;
            System{j}.waves.angle(1,:)=waves{j}.angle;
            if isfield(waves{j},'frequency')
                System{j}.waves.frequency=zeros(N,1);
                System{j}.waves.frequency(1,:)=waves{j}.frequency;
            end
        end
    end

    if ismember("currents", conditions)==true
        currents=currentsSet(ASVNum,env.currents,realStatesNow,0);
        for j=1:ASVNum
            System{j}.currents.earthSpeed=zeros(N,3);
            System{j}.currents.bodySpeed=zeros(N,3);
            System{j}.currents.earthSpeed(1,:)=currents{j}.earthSpeed;
            System{j}.currents.bodySpeed(1,:)=currents{j}.bodySpeed;
        end

    end





   % Whether each ASV has faults, observation, or control should be determined within subfunctions;  
   % Corresponding variables are initialized only if they exist—otherwise, the variables are not created  
    senNoise=cell(1,ASVNum);
    sensors=cell(1,ASVNum);
    if ismember("measureNoise", conditions)==true 
        senNoise=senNoi(ASVNum,Nx,N);
    end
    if ismember("sensorFault", conditions)==true
        sensors=senFau(ASVNum,Nx,realStatesNow,0);
    end
    for j=1:ASVNum
        if ismember("measureNoise", conditions)==true
            System{j}.measStates=zeros(N,Nx(1,j));
            System{j}.measNoise=senNoise{j};
            System{j}.measStates(1,:)=System{j}.realStates(1,:)+System{j}.measNoise(1,:);
            if ~isempty(sensors{j})
                System{j}.senStates=zeros(N,Nx(1,j));
                System{j}.senFaults=zeros(N,Nx(1,j));            
                System{j}.senStates(1,:)=sensors{j}.states;
                System{j}.senFaults(1,:)=sensors{j}.faults;
                System{j}.measStates(1,:)=(System{j}.measStates(1,:)+System{j}.senFaults(1,:)).*System{j}.senStates(1,:);
            end
        else
            if  ~isempty(sensors{j})
                System{j}.measStates=zeros(N,Nx(1,j));
                System{j}.senStates=zeros(N,Nx(1,j));
                System{j}.senFaults=zeros(N,Nx(1,j));
                System{j}.senStates(1,:)=sensors{j}.states;
                System{j}.senFaults(1,:)=sensors{j}.faults;
                System{j}.measStates(1,:)=(System{j}.realStates(1,:)+System{j}.senFaults(1,:)).*System{j}.senStates(1,:);
            end        
        end    
    end

    actuators=cell(1,ASVNum);
    if ismember("actuatorFault", conditions)==true        
        actuators=actFau(ASVNum,Nu,commandsNow,0);
        for j=1:ASVNum
            if ~isempty(actuators{j})
                System{j}.realInputs=zeros(N,Nu(1,j));
                System{j}.actStates=zeros(N,Nu(1,j));
                System{j}.actFaults=zeros(N,Nu(1,j));
                System{j}.actStates(1,:)=actuators{j}.states;
                System{j}.actFaults(1,:)=actuators{j}.faults;            
                System{j}.realInputs(1,:)=(System{j}.commands(1,:)+System{j}.actFaults(1,:)).*System{j}.actStates(1,:); 

            end
        end
    end

    if ismember("communicationFault", conditions)==true
        if ASVNum<2
            error('Inter-ASV communication cannot be set for a single ASV');
        end
        communications=comFau(ASVNum,realStatesNow,0);
        for j=1:ASVNum   
            System{j}.comRelSta=cell(1, ASVNum);
            System{j}.comStates=zeros(N,ASVNum);
            System{j}.comStates(1,:)=communications{j}.states;        
            System{j}.comDelays=zeros(N,ASVNum);
            System{j}.comDelays(1,:)=communications{j}.delays;
            for i=1:ASVNum    % Transmission content can be customized    
                System{j}.comRelSta{i}=zeros(N,Nx(i));
                System{j}.comRelSta{i}(1,:)=System{i}.realStates(1,:);         
            end
        end
    end    
end
end
