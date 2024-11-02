function q = inverseKinematics(directKinematics, jointLimits, desiredPosition, varargin)
    % Check if a desired rotation matrix is provided
    hasDesiredRotation = nargin > 3;
    if hasDesiredRotation
        desiredRotation = varargin{1};
    end

    % Number of joints
    numJoints = size(jointLimits, 2);
    
    % Initial guess for joint angles (midpoint of joint limits)
    q0 = (jointLimits(1, :) + jointLimits(2, :)) / 2;

    % Optimization options
    options = optimoptions('fmincon', 'Display', 'iter', 'SpecifyObjectiveGradient', true);

    % Define the cost function
    function [error_val, gradient] = costFunction(q)
        [positions, rotations, jacobians] = directKinematics(q);
        endEffectorPosition = positions{end};
        endEffectorRotation = rotations{end};
        endEffectorJacobian = jacobians{end};
        
        positionError = desiredPosition - endEffectorPosition;
        
        if hasDesiredRotation
            % Use rotation matrix to rotation vector conversion for error
            rotationErrorVector = 0.5 * [endEffectorRotation(3,2)-endEffectorRotation(2,3); ...
                                         endEffectorRotation(1,3)-endEffectorRotation(3,1); ...
                                         endEffectorRotation(2,1)-endEffectorRotation(1,2)];
            desiredRotationVector = 0.5 * [desiredRotation(3,2)-desiredRotation(2,3); ...
                                           desiredRotation(1,3)-desiredRotation(3,1); ...
                                           desiredRotation(2,1)-desiredRotation(1,2)];
                                       
            rotationError = desiredRotationVector - rotationErrorVector;
            totalError = [positionError; rotationError];
            
            % Jacobian for combined position and rotation error
            jacobian = [endEffectorJacobian(1:3, :); endEffectorJacobian(4:6, :)];
        else
            totalError = positionError;
            jacobian = endEffectorJacobian(1:3, :);  % Only position jacobian
        end
        
        % Sum of squared errors
        error_val = 0.5 * sum(totalError.^2);
        
        % Gradient of the cost function
        gradient = jacobian' * totalError;
    end

    % No linear constraints for this problem
    A = [];
    b = [];
    
    % Solve using fmincon
    q = fmincon(@costFunction, q0, A, b, [], [], jointLimits(1, :), jointLimits(2, :), [], options);
end
