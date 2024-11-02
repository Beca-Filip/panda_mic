function [limits1, limits2] = pandaJointLimits()
%PANDAJOINTLIMITS Returns the joint limits for a Panda robot.
%
% When two outputs are requested:
% [lowerLimits, upperLimits] = pandaJointLimits();
% lowerLimits: A 1x7 vector containing the lower joint limits for each joint.
% upperLimits: A 1x7 vector containing the upper joint limits for each joint.
%
% When one output is requested:
% limits = pandaJointLimits();
% limits: A 2x7 matrix where the first row contains the lower joint limits 
% and the second row contains the upper joint limits.
%
% Example:
% [lower, upper] = pandaJointLimits();
% This will return the lower and upper joint limits for the Panda robot.
% limits = pandaJointLimits();
% This will return a 2x7 matrix of joint limits.

% Define the lower joint limits for joints 1 through 7
lowerLimits = deg2rad([-166, -101, -166, -176, -166, -1, -166]);

% Define the upper joint limits for joints 1 through 7
upperLimits = deg2rad([166, 101, 166, -4, 166, 215, 166]);

if nargout == 1
    limits1 = [lowerLimits; upperLimits];
else
    limits1 = lowerLimits;
    limits2 = upperLimits;
end
end
