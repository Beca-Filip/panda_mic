clear all;
close all;
clc;

% Initial Values
    % Sphere parameters
    initValues.sphereCenter_val = [.75; 0; .25]; % position [m]
    initValues.sphereRadius_val = .5; % [m]
    initValues.sphereAngularGridSpacing_val = 60; % [deg]
    % Collision sphere
    initValues.collisionSphereRadius_val = 0.25; % [m]

    % Sphere grid points
    sphereGridPoints = initValues.sphereCenter_val + createSphereGridpoints(initValues.sphereRadius_val, initValues.sphereAngularGridSpacing_val, initValues.sphereAngularGridSpacing_val);
    initValues.gridPoints_val = sphereGridPoints;
    initValues.N = size(sphereGridPoints, 2);

    % Robot initial config
    initValues.n = 7;
    initValues.robotConfig_val = zeros(7, size(sphereGridPoints, 2));
    % Set the 4th joint to -5 deg
    initValues.robotConfig_val(4, :) = deg2rad(-5);
    initValues.jointLimits_val = pandaJointLimits();

    % Stick parameters
    initValues.lstick1_val = 0.3; % [m]
    initValues.lstick2_val = 0.3; % [m]
    initValues.qstick1_val = deg2rad(-60);
    initValues.qstick2_val = deg2rad(60);
    stickParams = [ ...
        initValues.lstick1_val;
        initValues.lstick2_val;
        initValues.qstick1_val;
        initValues.qstick2_val;
    ];
    
% Box constraints
    % Sphere center and radius box constraints
%     initValues.sphereCenterLowerLimit_val = [.50; -.50; 0];
%     initValues.sphereCenterUpperLimit_val = [2; .50; 1];
%     initValues.sphereRadiusLowerLimit_val = .30;
%     initValues.sphereRadiusUpperLimit_val = 1;
    initValues.sphereCenterLowerLimit_val = initValues.sphereCenter_val;
    initValues.sphereCenterUpperLimit_val = initValues.sphereCenter_val;
    initValues.sphereRadiusLowerLimit_val = initValues.sphereRadius_val;
    initValues.sphereRadiusUpperLimit_val = initValues.sphereRadius_val;
    % Stick parameters box constraints(l1, l2, q1, q2)
%     initValues.stickParamsLowerLimit_val = [.05; .05; -pi/3; -pi/3];
%     initValues.stickParamsUpperLimit_val = [1; 1; pi/3; pi/3];
    initValues.stickParamsLowerLimit_val = stickParams;
    initValues.stickParamsUpperLimit_val = stickParams;
    
% Create object
tpo = taskParameterOptimizer(initValues);
tpo = tpo.forwardKinematics();
tpo = tpo.calculateDistances();
tpo = tpo.formOptimizationProblem();
tpo = tpo.initialize();

%%
tpo = tpo.setDefaultIpoptOptions();

sol = tpo.opti.solve();
% [posSticks, rotSticks] = stickKinematics(pos{8}, rot{8}, lstick1, lstick2, qstick1, qstick2);
% 
% 
% figure('WindowState', 'maximized');
% hold on;
% robotPlot(pos, rot);
% hold on;
% stickPlot(posSticks, rotSticks);
% hold on;
% plot3(sphereGridPoints(1, :), sphereGridPoints(2, :), sphereGridPoints(3, :), 'bo');
% xlim([-1, 1.5]);
% ylim([-1, 1]);
% zlim([-1, 1.5]);
% view(30, 30)