clear all;
close all;
clc;

% Sphere parameters
ps = [.75; 0; .5]; % position [m]
rs = .5; % [m]
angStep = 15; % [deg]

% Sphere grid points
sphereGridPoints = ps + createSphereGridpoints(rs, angStep, angStep);

% Robot initial config
q = zeros(7, 1);
[pos, rot, jac] = pandaAutomaticKinematics(q);

% Stick parameters
lstick1 = 0.3; % [m]
lstick2 = 0.3; % [m]
qstick1 = deg2rad(-60);
qstick2 = deg2rad(120);
[posSticks, rotSticks] = stickKinematics(pos{8}, rot{8}, lstick1, lstick2, qstick1, qstick2);

% Stick params in a vector
stickParams = [lstick1; lstick2; qstick1; qstick2];
sphereParams = [ps; rs/2];

% Initialize VideoMaker
vm = VideoMaker('PandaMic', 2);

% Initialize structures for data storage
Q_IK = zeros(7, size(sphereGridPoints, 2));

% Initialize figure
figure('WindowState', 'maximized');

for k = 1 : size(sphereGridPoints, 2)
    % Clear axes
    cla;
    
    % Robot config for ik
    if k == 1
        % No initial guess
        q_ik = inverseStickKinematicsConstrainedCasadi(@pandaAutomaticKinematics, @stickKinematics, stickParams, pandaJointLimits(), sphereParams, sphereGridPoints(:, k));
    else
        % Yes initial guess
        q_ik = inverseStickKinematicsConstrainedCasadi(@pandaAutomaticKinematics, @stickKinematics, stickParams, pandaJointLimits(), sphereParams, sphereGridPoints(:, k), q_ik);
    end
    
    [pos_ik, rot_ik, jac_ik] = pandaAutomaticKinematics(q_ik);
    [posSticks_ik, rotSticks_ik] = stickKinematics(pos_ik{8}, rot_ik{8}, lstick1, lstick2, qstick1, qstick2);

    hold on;
    robotPlot(pos_ik, rot_ik);
    hold on;
    stickPlot(posSticks_ik, rotSticks_ik);
    hold on;
    plot3DSphere(ps, rs/2);
    hold on;
    plot3(sphereGridPoints(1, :), sphereGridPoints(2, :), sphereGridPoints(3, :), 'bo', 'DisplayName', 'Measurement Points');
    xlim([-1, 1.5]);
    ylim([-1, 1]);
    zlim([-1, 1.5]);
    view(30, 30)
    legend('Location', 'bestoutside', 'Interpreter', 'latex', 'FontSize', 15);
    title(sprintf('Measurement point %5d/%d.', k, size(sphereGridPoints, 2)), 'Interpreter', 'latex', 'FontSize', 15);
    
    % Capture current figure
    vm.capture();
    
    % Store the joint angles
    Q_IK(:, k) = q_ik;
end

% Render video
vm.render();
% Save joint angles
save('pandaInverseStickKinematics.mat', 'Q_IK');