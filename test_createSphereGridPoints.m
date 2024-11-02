clear all
close all
clc
 
% Sphere radius
rs = 0.5;
stepPhiDeg = 15;
stepThetaDeg = 15;

% Sphere 
sphereGridPoints = createSphereGridpoints(rs, stepPhiDeg, stepThetaDeg);

figure;
plot3(sphereGridPoints(1, :), sphereGridPoints(2, :), sphereGridPoints(3, :), 'bo');
