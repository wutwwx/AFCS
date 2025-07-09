function env = generateEnvironment(MapName, obsTypes, steps, dt, doPlot)
%GENERATEENVIRONMENT   Create a navigation environment with static and dynamic obstacles.
%
%   env = generateEnvironment(MapName, obsTypes, steps, dt, doPlot)
%   Generates a structured environment for simulation, including navigation area,
%   static obstacles (manual/random), dynamic obstacles (with trajectories), currents, winds, and waves.
%
%   Inputs:
%       MapName   - String. Name of the pre-defined map, e.g., 'openWater'
%       obsTypes  - Cell array. Types of obstacles, e.g., {'manual_static','rand_static','manual_dynamic'}
%       steps     - Integer. Number of simulation time steps for dynamic obstacles
%       dt        - Scalar.   Sampling interval (s) for dynamic obstacle trajectories
%       doPlot    - Logical (optional). Whether to visualize the environment (default: true)
%
%   Output:
%       env       - Struct containing fields: navArea, staticObs, manual_static, rand_static,
%                  manual_dynamic, currents, winds, waves, etc.
%
%   Example:
%       env = generateEnvironment('openWater', {'manual_static','rand_static'}, N, Ts, true);
%
%   Author: Wenxiang Wu (with ChatGPT enhancement)
%   Date:   2025-06-19

if nargin < 5, doPlot = true; end

env = struct("manual_static",[],"rand_static",[],"staticObs",[],"manual_dynamic",[], ...
             "navArea",[],"currents",[],"winds",[],"waves",[]);

if ~isempty(MapName)
    switch MapName
        case "openWater"
            naviMap = openWater();
    end
    env.navArea   = naviMap.navArea;
    env.currents  = naviMap.currents;
    env.winds     = naviMap.winds;
    env.waves     = naviMap.waves;
end

if ~isempty(obsTypes)
    allowed_strings = ["manual_static", "rand_static", "manual_dynamic"];
    invalid_obstacles = obsTypes(~ismember(obsTypes, allowed_strings));
    if ~isempty(invalid_obstacles)
        error('Error: The following obstacle types are not allowed: %s', strjoin(invalid_obstacles, ', '));
    end

    navArea = [0 0; 30 0; 30 30; 0 30];
    if ~isempty(MapName)
        if isempty(naviMap.navArea)
            env.navArea = navArea;
        else
            navArea = naviMap.navArea;
        end
    else
        env.navArea = navArea;
    end

    % ==================== 1. Visualization ======================
    if doPlot
        figure(99); clf; hold on; axis equal; box on; grid on;
        fill(navArea(:,1), navArea(:,2), [0.9 0.9 0.9], 'FaceAlpha', .5, 'EdgeColor', 'none');
        xlim([navArea(1,2) navArea(3,2)]); ylim([navArea(1,1) navArea(3,1)]);
        xlabel('Y / m'); ylabel('X / m');
        title('Navigation Environment');
    end

    %% 2. Dynamic obstacle generation
    dynSegs = [];
    dynSafeR = [];
    if ismember("manual_dynamic", obsTypes)
        dynStates = dynamicObstacle(steps, dt);
        env.manual_dynamic = struct('TS', []);
        for j = 1:numel(dynStates)
            traj = dynStates{j}.path;
            spdHist = dynStates{j}.spd_hist;
            hdgHist = dynStates{j}.hdg_hist;
            env.manual_dynamic.TS{j}.Pos = traj;
            env.manual_dynamic.TS{j}.Spd = spdHist;
            env.manual_dynamic.TS{j}.Hdg = hdgHist;
            env.manual_dynamic.R(1,j) = 0.5*dynStates{j}.ship_len;
            p0 = traj(1, :);  pN = traj(end, :);
            dynSegs = [dynSegs; p0 pN]; %#ok<AGROW>
            dynSafeR = [dynSafeR, 0.5*dynStates{j}.ship_len];
        end
    end

    %% 3. Static obstacles (prevent overlap and avoid dynamic obstacles)
    staticCenters = [];
    staticRadii   = [];
    manualObsCell = {};

    %% 4. Manual static obstacles
    if ismember("manual_static", obsTypes)
        manualObsCell = staticObstacleManual(navArea);
        for k = 1:numel(manualObsCell)
            obs = manualObsCell{k};
            ctr = obs.center; R = obs.radius;
            if ismember("manual_dynamic", obsTypes)
                if violateDyn(ctr,R)
                    warning('Manual static %d too close to dynamic, discarded.',k);
                elseif ~isStaticOK(ctr,R)
                    warning('Manual static %d overlaps another static, discarded.',k);
                else
                    addStatic(obs.point, obs.center, R);
                end
            else
                if ~isStaticOK(ctr,R)
                    warning('Manual static %d overlaps another static, discarded.',k);
                else
                    addStatic(obs.point, obs.center, R);
                end
            end
        end
    end

    %% 5. Random static obstacles
    if ismember("rand_static", obsTypes)
        needN = 3; maxTry = 30; got = 0; tries = 0;
        avoidPts = zeros(0,3);
        for k = 1:numel(env.staticObs)
            avoidPts(end+1,:) = [env.staticObs{k}.center, env.staticObs{k}.Rad];
        end
        while got < needN && tries < maxTry
            tries = tries + 1;
            raw = staticObstacleRand(navArea, avoidPts);
            for m = 1:numel(raw)
                pts = raw{m}.points; R = raw{m}.radius; ctr = raw{m}.center;
                if ismember("manual_dynamic", obsTypes)
                    if ~violateDyn(ctr,R) && isStaticOK(ctr,R)
                        addStatic(pts, ctr, R);
                        got = got + 1;
                        if got >= needN, break; end
                    end
                else
                    if isStaticOK(ctr,R)
                        addStatic(pts, ctr, R);
                        got = got + 1;
                        if got >= needN, break; end
                    end
                end
            end
        end
        fprintf('Random static obtained %d/%d (tries %d)\n', got, needN, tries);
    end

    %% 6. Output all static obstacles
    nMan = numel(env.staticObs);
    env.manual_static = env.staticObs(1:nMan);
    env.rand_static   = env.staticObs(nMan+1:end);

    % ========== Dynamic obstacle animation ==========
    if ismember("manual_dynamic", obsTypes) && doPlot
        dynShips = env.manual_dynamic.TS;
        Nship = numel(dynShips);
        Tmax = size(dynShips{1}.Pos,1);

        trajHandles = gobjects(Nship,1);
        shipHandles = gobjects(Nship,1);
        for j = 1:Nship
            posHist = dynShips{j}.Pos(1, :);
            trajHandles(j) = plot(posHist(2), posHist(1), '-', 'Color', [0 0 1]*0.7, 'LineWidth', 1.5);
        end

        maxFrames = 200; % Max animation frames
        pause_time = 0.025; % Pause per frame
        frameIdx = unique(round(linspace(1, Tmax, min(maxFrames,Tmax))));
        for t = frameIdx
            % Draw trajectory
            for j = 1:Nship
                xHist = dynShips{j}.Pos(1:t, 2);
                yHist = dynShips{j}.Pos(1:t, 1);
                set(trajHandles(j), 'XData', xHist, 'YData', yHist);
            end
            % Draw ship shape
            for j = 1:Nship
                if isgraphics(shipHandles(j)), delete(shipHandles(j)); end
            end
            for j = 1:Nship
                pos = dynShips{j}.Pos(t,:);
                hdg = dynShips{j}.Hdg(t);
                theta = [hdg 0 0];
                scale = 0.8; color = [0.0 0.5 1];
                shipHandles(j) = shipDisplay3(theta, pos(2), pos(1), 0, scale, color);
            end
            drawnow limitrate;
            pause(pause_time);
        end
    end

    if doPlot, hold off; end
end

% ------- Internal helper functions -------

function bad = violateDyn(center, radius)
    bad = false;
    for seg = 1:size(dynSegs,1)
        a = dynSegs(seg,1:2);  b = dynSegs(seg,3:4);
        d = point2seg(center,a,b);
        safeRange = radius + dynSafeR(seg);
        if d <= safeRange
            bad = true; return
        end
    end
end

function d = point2seg(p, A, B)
    v = B - A; w = p - A;
    t = max(0, min(1, dot(w,v)/dot(v,v)));
    proj = A + t*v;
    d = norm(p - proj);
end

function ok = isStaticOK(center, radius)
    ok = true;
    for k = 1:size(staticCenters,1)
        if norm(center - staticCenters(k,:)) <= (radius + staticRadii(k))
            ok = false; return;
        end
    end
end

function addStatic(pts, center, radius)
    staticCenters(end+1,:) = center;
    staticRadii(end+1)   = radius;
    idx = numel(env.staticObs)+1;
    env.staticObs{idx}.Pos = pts;
    env.staticObs{idx}.Rad = radius;
    env.staticObs{idx}.center = center;
    if doPlot
        % Draw static obstacle in figure 99
        figure(99);
        fill(pts(:,2), pts(:,1), 'k', 'FaceAlpha', .5, 'EdgeColor', 'k');
    end
end

end
