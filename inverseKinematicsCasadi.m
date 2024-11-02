function q_optimal = inverseKinematicsCasadi(directKinematics, jointLimits, desiredPosition, varargin)

    % Check if a desired rotation matrix is provided
    hasDesiredRotation = nargin > 3;
    if hasDesiredRotation
        desiredRotation = varargin{1};
    end

    % Number of joints
    numJoints = size(jointLimits, 2);
    
    % Create an optimization object
    opti = casadi.Opti();

    % Define decision variables (joint angles)
    q = opti.variable(numJoints, 1);

    % Define constraints (joint limits)
    opti.subject_to(jointLimits(1, :)' <= q <= jointLimits(2, :)');

    % Retrieve the direct kinematics outputs using the decision variables
    [positions, rotations, ~] = directKinematics(q);
    
    % Compute position error
    positionError = desiredPosition - positions{end};
    opti.minimize(sum(positionError.^2));  % Minimize squared position error

    % If orientation is also specified, compute orientation error and add to cost
    if hasDesiredRotation
        endEffectorRotation = rotations{end};
        rotationErrorVector = 0.5 * [endEffectorRotation(3,2)-endEffectorRotation(2,3); ...
                                     endEffectorRotation(1,3)-endEffectorRotation(3,1); ...
                                     endEffectorRotation(2,1)-endEffectorRotation(1,2)];
        desiredRotationVector = 0.5 * [desiredRotation(3,2)-desiredRotation(2,3); ...
                                       desiredRotation(1,3)-desiredRotation(3,1); ...
                                       desiredRotation(2,1)-desiredRotation(1,2)];
                                   
        rotationError = desiredRotationVector - rotationErrorVector;
        opti.minimize(sum(rotationError.^2));  % Add squared rotation error to cost
    end

    % Set initial guess for joint angles
    opti.set_initial(q, (jointLimits(1, :) + jointLimits(2, :))' / 2);

    % Solve the problem using 'ipopt'
    opts = struct('ipopt', struct('print_level', 5, 'tol', 1e-6));
    opti.solver('ipopt', opts);

    % Get the optimal solution
    sol = opti.solve();
    q_optimal = sol.value(q);

end
