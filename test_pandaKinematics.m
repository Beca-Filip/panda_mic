clear all;
close all;
clc;

% Sphere parameters
ps = [.75; 0; .25]; % position [m]
rs = .5; % [m]
angStep = 60; % [deg]

% Sphere grid points
sphereGridPoints = ps + createSphereGridpoints(rs, angStep, angStep);

% Robot initial config
q = zeros(7, 1);
[pos, rot, jac] = pandaAutomaticKinematics(q);

% Stick parameters
lstick1 = 0.3; % [m]
lstick2 = 0.3; % [m]
qstick1 = deg2rad(-60);
qstick2 = deg2rad(60);
[posSticks, rotSticks] = stickKinematics(pos{8}, rot{8}, lstick1, lstick2, qstick1, qstick2);


figure('WindowState', 'maximized');
hold on;
robotPlot(pos, rot);
hold on;
stickPlot(posSticks, rotSticks);
hold on;
plot3(sphereGridPoints(1, :), sphereGridPoints(2, :), sphereGridPoints(3, :), 'bo');
xlim([-1, 1.5]);
ylim([-1, 1]);
zlim([-1, 1.5]);
view(30, 30)