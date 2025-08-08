clc; clear; close all;

%% Environment Parameters
roomLength = 10; roomWidth = 10; roomHeight = 3;
startPos = [1, 1, 1];
endPos = [9, 9, 1];
dt = 0.1; T = 60; numSteps = T / dt;
%% lidara sensor
%% collision avoidance mattrices
%% Geo-Fence Definition (UAV must stay inside this box)
geoFenceMin = [1 1 0.5];
geoFenceMax = [9 9 2.5];

%% Waypoints: Start, Two Mids, End
waypoints = [
    startPos;
    2 8 1.2;
    8 4 1.4;
    endPos
];
currentWaypoint = 2;

%% Visualization Setup
figure;
axis equal; grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([0 roomLength]); ylim([0 roomWidth]); zlim([0 roomHeight]);
view(3); hold on;
title('UAV Navigation with Geo-Fencing');

% Floor and Ceiling
[Xf, Yf] = meshgrid(0:0.5:roomLength, 0:0.5:roomWidth);
Zf = zeros(size(Xf));
surf(Xf, Yf, Zf, 'FaceColor', [0.85 0.85 0.85], 'EdgeColor', 'none');
Zc = roomHeight * ones(size(Xf));
surf(Xf, Yf, Zc, 'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.05);

% Walls
wallColor = [0.8 0.8 0.9]; wallAlpha = 0.3;
fill3([0 0 0 0], [0 roomWidth roomWidth 0], [0 0 roomHeight roomHeight], wallColor, 'FaceAlpha', wallAlpha);
fill3([roomLength roomLength roomLength roomLength], [0 roomWidth roomWidth 0], [0 0 roomHeight roomHeight], wallColor, 'FaceAlpha', wallAlpha);
fill3([0 roomLength roomLength 0], [0 0 0 0], [0 0 roomHeight roomHeight], wallColor, 'FaceAlpha', wallAlpha);
fill3([0 roomLength roomLength 0], [roomWidth roomWidth roomWidth roomWidth], [0 0 roomHeight roomHeight], wallColor, 'FaceAlpha', wallAlpha);

% === Visualize Geo-Fence ===
fill3([geoFenceMin(1) geoFenceMax(1) geoFenceMax(1) geoFenceMin(1)], [geoFenceMin(2) geoFenceMin(2) geoFenceMax(2) geoFenceMax(2)], [geoFenceMin(3) geoFenceMin(3) geoFenceMin(3) geoFenceMin(3)], 'b', 'FaceAlpha', 0.05, 'EdgeColor', 'b', 'LineStyle', '--');
fill3([geoFenceMin(1) geoFenceMax(1) geoFenceMax(1) geoFenceMin(1)], [geoFenceMin(2) geoFenceMin(2) geoFenceMax(2) geoFenceMax(2)], [geoFenceMax(3) geoFenceMax(3) geoFenceMax(3) geoFenceMax(3)], 'b', 'FaceAlpha', 0.05, 'EdgeColor', 'b', 'LineStyle', '--');

% Geo-Fence edges
corners = [
    geoFenceMin;
    geoFenceMax(1), geoFenceMin(2), geoFenceMin(3);
    geoFenceMax(1), geoFenceMax(2), geoFenceMin(3);
    geoFenceMin(1), geoFenceMax(2), geoFenceMin(3)
];
cornersTop = corners; cornersTop(:,3) = geoFenceMax(3);
for i = 1:4
    plot3([corners(i,1), cornersTop(i,1)], [corners(i,2), cornersTop(i,2)], [corners(i,3), cornersTop(i,3)], 'b--');
end

% Waypoints
scatter3(startPos(1), startPos(2), startPos(3), 120, 'y', 'filled');
scatter3(endPos(1), endPos(2), endPos(3), 120, 'r', 'filled');
scatter3(waypoints(2:3,1), waypoints(2:3,2), waypoints(2:3,3), 100, 'g', 'filled');

%% UAV Initialization
uav.pos = startPos; uav.path = uav.pos;
[uavBodyX, uavBodyY, uavBodyZ] = cylinder([0.2 0.2], 12);
uavBodyZ = uavBodyZ * 0.1;
uavBody = surf(uavBodyX + uav.pos(1), uavBodyY + uav.pos(2), uavBodyZ + uav.pos(3)-0.05, 'FaceColor', 'r', 'EdgeColor', 'none');

armLen = 0.3; rotorRadius = 0.05;
for i = 1:4
    angle = pi/4 + (i-1)*pi/2;
    endArm = uav.pos + armLen * [cos(angle), sin(angle), 0];
    armLines(i) = plot3([uav.pos(1), endArm(1)], [uav.pos(2), endArm(2)], [uav.pos(3), endArm(3)], 'k', 'LineWidth', 2);
    [rx, ry, rz] = cylinder(rotorRadius, 12);
    rz = rz * 0.02;
    rotors(i) = surf(rx + endArm(1), ry + endArm(2), rz + endArm(3), 'FaceColor', 'k', 'EdgeColor', 'none');
end

%% Obstacles
numObstacles = 8; minGap = 1.5;
obstacles = struct(); positions = [];
for i = 1:numObstacles
    while true
        pos = rand(1,3) .* [roomLength-1, roomWidth-1, 1];
        if isempty(positions) || all(vecnorm(positions - pos, 2, 2) > minGap)
            break;
        end
    end
    positions = [positions; pos];
    vel = (rand(1,2) - 0.5) * 0.3; isBox = mod(i,2) == 0;

    if isBox
        boxSize = 0.4 + rand(1,3) .* [0.4, 0.3, 1.0];
        [X, Y, Z] = ndgrid([-0.5 0.5], [-0.5 0.5], [0 1]);
        X = X * boxSize(1); Y = Y * boxSize(2); Z = Z * boxSize(3);
        theta = rand() * 2*pi;
        R = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
        points = [X(:), Y(:), Z(:)] * R' + pos;
        faces = [1 2 4 3; 5 6 8 7; 1 2 6 5; 2 4 8 6; 4 3 7 8; 3 1 5 7];
        obstacles(i).type = 'box';
        obstacles(i).plot = patch('Vertices', points, 'Faces', faces, 'FaceColor', rand(1,3), 'EdgeColor', 'k', 'FaceAlpha', 0.6);
    else
        [xc, yc, zc] = cylinder(0.3 + rand()*0.2, 20);
        h = 1.0 + rand(); zc = zc * h;
        obstacles(i).type = 'cylinder';
        obstacles(i).plot = surf(xc + pos(1), yc + pos(2), zc + pos(3), 'FaceColor', rand(1,3), 'EdgeColor', 'none', 'FaceAlpha', 0.5);
    end
    obstacles(i).pos = pos; obstacles(i).vel = [vel, 0]; obstacles(i).radius = 0.4;
end

%% Main Loop
for t = 1:numSteps
    % Obstacle Movement
    for i = 1:numObstacles
        obstacles(i).pos = obstacles(i).pos + obstacles(i).vel * dt;
        for d = 1:2
            if obstacles(i).pos(d) < 0 || obstacles(i).pos(d) > roomLength
                obstacles(i).vel(d) = -obstacles(i).vel(d);
            end
        end
        if strcmp(obstacles(i).type, 'cylinder')
            [xc, yc, zc] = cylinder(0.3, 20);
            set(obstacles(i).plot, 'XData', xc + obstacles(i).pos(1), 'YData', yc + obstacles(i).pos(2), 'ZData', zc + obstacles(i).pos(3));
        end
    end

    % Path Planning with Geo-Fencing
    avoidance = [0 0 0];
    for i = 1:numObstacles
        diff = uav.pos - obstacles(i).pos; dist = norm(diff);
        if dist < 1.5
            avoidance = avoidance + diff / dist^2;
        end
    end
    target = waypoints(currentWaypoint,:);
    dir = (target - uav.pos); dir = dir / norm(dir);
    vel = dir + 1.5 * avoidance;
    vel = vel / norm(vel) * 0.1;

    % Enforce Geo-Fence on UAV
    nextPos = uav.pos + vel;
    for d = 1:3
        nextPos(d) = max(geoFenceMin(d), min(geoFenceMax(d), nextPos(d)));
    end
    uav.pos = nextPos;
    uav.path(end+1,:) = uav.pos;

    if norm(uav.pos - target) < 0.3 && currentWaypoint < size(waypoints,1)
        currentWaypoint = currentWaypoint + 1;
    end

    % UAV Update
    set(uavBody, 'XData', uavBodyX + uav.pos(1), 'YData', uavBodyY + uav.pos(2), 'ZData', uavBodyZ + uav.pos(3)-0.05);
    for i = 1:4
        angle = pi/4 + (i-1)*pi/2;
        endArm = uav.pos + armLen * [cos(angle), sin(angle), 0];
        set(armLines(i), 'XData', [uav.pos(1), endArm(1)], 'YData', [uav.pos(2), endArm(2)], 'ZData', [uav.pos(3), endArm(3)]);
        set(rotors(i), 'XData', rx + endArm(1), 'YData', ry + endArm(2), 'ZData', rz + endArm(3));
    end

    drawnow;
    if norm(uav.pos - endPos) < 0.3
        disp('Goal Reached!'); break;
    end
end

plot3(uav.path(:,1), uav.path(:,2), uav.path(:,3), 'g-', 'LineWidth', 2);

%% Legend
legend([plot3(NaN,NaN,NaN,'yo','MarkerFaceColor','y'), ...
        plot3(NaN,NaN,NaN,'go','MarkerFaceColor','g'), ...
        plot3(NaN,NaN,NaN,'ro','MarkerFaceColor','r'), ...
        plot3(NaN,NaN,NaN,'b--'), ...
        plot3(NaN,NaN,NaN,'ks','MarkerFaceColor','y'), ...
        plot3(NaN,NaN,NaN,'bo','MarkerFaceColor','b'), ...
        plot3(NaN,NaN,NaN,'g-')], ...
        {'Start Point', 'Mid Waypoints', 'End Point', 'Geo-Fence', 'Box Obstacle', 'Cylinder Obstacle', 'UAV Path'}, ...
         'Location', 'northeastoutside');f