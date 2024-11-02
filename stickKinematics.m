function [posSticks, rotSticks] = stickKinematics(posEE, rotEE, lstick1, lstick2, qstick1, qstick2)
%STICKKINEMATICS Compute the position and orientation of two sticks attached to a robot's end effector.
%
%   INPUTS:
%       posEE    - Position of the robot's end effector [3x1 vector]
%       rotEE    - Rotation matrix of the robot's end effector [3x3 matrix]
%       lstick1  - Length of the first stick [scalar]
%       lstick2  - Length of the second stick [scalar]
%       qstick1  - Rotation angle (around y-axis) for the first stick [scalar, radians]
%       qstick2  - Rotation angle (around y-axis) for the second stick [scalar, radians]
%
%   OUTPUTS:
%       posSticks - Cell array containing positions of the end effector, 
%                   end of the first stick, and end of the second stick [{3x1}; {3x1}; {3x1}]
%       rotSticks - Cell array containing rotation matrices of the end effector, 
%                   first stick, and second stick [{3x3}; {3x3}; {3x3}]
%
%   DESCRIPTION:
%       Given the position and rotation matrix of a robot's end effector, 
%       and the lengths and angles of two attached sticks, this function 
%       computes the positions and orientations of the ends of the two sticks.
%

% Initialize stick positions and rotation matrices
% 'class' ensures the data type consistency of these variables with the lengths lstick1 and lstick2.
ps1 = zeros(3, 1, class(lstick1)); % Position vector for the first stick, initialized to [0; 0; 0]
ps2 = zeros(3, 1, class(lstick2)); % Position vector for the second stick, initialized to [0; 0; 0]

Rs1 = eye(3, class(qstick1)); % Rotation matrix for the first stick, initialized to identity matrix
Rs2 = eye(3, class(qstick2)); % Rotation matrix for the second stick, initialized to identity matrix

% Update the z-coordinates with stick lengths
ps1(3) = lstick1;
ps2(3) = lstick2;

% Calculate the rotation matrix for the first stick using its angle 'qstick1'
% This is essentially a rotation about the y-axis
Rs1(1, 1) = cos(qstick1); 
Rs1(1, 3) = -sin(qstick1);
Rs1(3, 1) = sin(qstick1);
Rs1(3, 3) = cos(qstick1);

% Calculate the rotation matrix for the second stick using its angle 'qstick2'
% Similarly, this is a rotation about the y-axis
Rs2(1, 1) = cos(qstick2); 
Rs2(1, 3) = -sin(qstick2);
Rs2(3, 1) = sin(qstick2);
Rs2(3, 3) = cos(qstick2);

% Compute the global rotation and position for the first stick
rotStick1 = rotEE * Rs1;         % Rotate the end effector's rotation by the stick's rotation
posStick1 = posEE + rotStick1 * ps1; % Position of the end of the first stick

% Compute the global rotation and position for the second stick
rotStick2 = rotStick1 * Rs2;         % Rotate the first stick's rotation by the second stick's rotation
posStick2 = posStick1 + rotStick2 * ps2; % Position of the end of the second stick

% Consolidate the positions and rotations of the end effector, first stick, and second stick into cell arrays
posSticks = {posEE, posStick1, posStick2};
rotSticks = {rotEE, rotStick1, rotStick2};

end
