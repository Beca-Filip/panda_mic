function [positions, rotations, jacobians] = pandaAutomaticKinematics(q)
% pandaAutomaticKinematics - Compute the kinematics of pandaAutomatic based on joint configurations
%
% INPUT:
%   q - Vector of joint configurations. Dimension: (n x 1)
%
% OUTPUT:
%   positions - Cell array of positions of each joint in the robot. Dimension: (1 x n)
%   rotations - Cell array of rotation matrices for each joint in the robot. Dimension: (1 x n)
%   jacobians - Cell array of Jacobians for each joint in the robot. Dimension: (1 x n)
%
% Created by createKinematicsFunction.m

n = 8; 
% Initialize cell arrays for positions, rotations, and jacobians
positions = cell(1, n);
rotations = cell(1, n);
jacobians = cell(1, n);

% Initial transformation matrix
T = eye(4, class(q));

% Declare DH vectors
theta = zeros(1, n, class(q));
d = zeros(1, n, class(q));
a = zeros(1, n, class(q));
alpha = zeros(1, n, class(q));

theta(1) = q(1);
d(1) = 0.3330000000000000;
a(1) = 0.0000000000000000;
alpha(1) = 0.0000000000000000;

theta(2) = q(2);
d(2) = 0.0000000000000000;
a(2) = 0.0000000000000000;
alpha(2) = -1.5707963267948966;

theta(3) = q(3);
d(3) = 0.3160000000000000;
a(3) = 0.0000000000000000;
alpha(3) = 1.5707963267948966;

theta(4) = q(4);
d(4) = 0.0000000000000000;
a(4) = 0.0825000000000000;
alpha(4) = 1.5707963267948966;

theta(5) = q(5);
d(5) = 0.3840000000000000;
a(5) = -0.0825000000000000;
alpha(5) = -1.5707963267948966;

theta(6) = q(6);
d(6) = 0.0000000000000000;
a(6) = 0.0000000000000000;
alpha(6) = 1.5707963267948966;

theta(7) = q(7);
d(7) = 0.0000000000000000;
a(7) = 0.0880000000000000;
alpha(7) = 1.5707963267948966;

theta(8) = 0.0000000000000000;
d(8) = 0.1070000000000000;
a(8) = 0.0000000000000000;
alpha(8) = 0.0000000000000000;

% isRotational is a binary vector indicating the type of each joint:
% 1 indicates a rotational joint, 0 indicates a translational joint
isRotational = [1  1  1  1  1  1  1  0];

% variableJoints is a binary vector indicating if each joint is variable:
% 1 indicates a joint that can vary (either rotational or translational)
% 0 indicates it's a constant joint
variableJoints = [1  1  1  1  1  1  1  0];


% Compute the transformation matrices using the modified DH vectors
for i = 1:n
    T = T * [cos(theta(i)) -sin(theta(i)) 0 a(i);
              sin(theta(i))*cos(alpha(i)) cos(theta(i))*cos(alpha(i)) -sin(alpha(i)) -sin(alpha(i))*d(i);
              sin(theta(i))*sin(alpha(i)) cos(theta(i))*sin(alpha(i)) cos(alpha(i)) cos(alpha(i))*d(i);
              0 0 0 1];

    % Store position and rotation for the current joint
    positions{i} = T(1:3, 4);
    rotations{i} = T(1:3, 1:3);

end
% Compute Jacobians for each link
for idx = 1:n
    J = zeros(6, sum(variableJoints),class(q));
    for j = 1:idx
        if variableJoints(j) == 1
            if j == 1
                p_j = zeros(3, 1, class(q));  % Origin for base frame
                z_j = [zeros(2, 1, class(q)); ones(1, 1, class(q))];  % z-axis for base frame
            else
                p_j = positions{j};
                z_j = rotations{j}(:, 3);
            end
            if isRotational(j) == 1
                J(1:3, j) = cross(z_j, positions{idx} - p_j);
                J(4:6, j) = z_j;
            else
                J(1:3, j) = z_j;
                J(4:6, j) = zeros(3, 1, class(q));
            end
        end
    end
    jacobians{idx} = J;
end

end