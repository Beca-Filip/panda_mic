function q_optimal = inverseStickKinematicsCasadi(directKinematics, stickDirectKinematics, stickParams, jointLimits, desiredPosition, varargin)

    % Extract stick parameters
    lstick1 = stickParams(1);
    lstick2 = stickParams(2);
    qstick1 = stickParams(3);
    qstick2 = stickParams(4);

    % Check if a desired rotation matrix is provided
    hasDesiredRotation = nargin > 5;
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

    % Compute the stick's kinematics
    [stickPositions, stickRotations] = stickDirectKinematics(positions{end}, rotations{end}, lstick1, lstick2, qstick1, qstick2);
    
    % Compute position error for the stick-end frame
    positionError = desiredPosition - stickPositions{end};
    opti.minimize(sum(positionError.^2));  % Minimize squared position error

    % If orientation is also specified, compute orientation error for the stick-end frame and add to cost
    if hasDesiredRotation
        stickEndRotation = stickRotations{end};
        rotationErrorVector = 0.5 * [stickEndRotation(3,2)-stickEndRotation(2,3); ...
                                     stickEndRotation(1,3)-stickEndRotation(3,1); ...
                                     stickEndRotation(2,1)-stickEndRotation(1,2)];
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
