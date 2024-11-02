classdef taskParameterOptimizer
    %taskParameterOptimizer - A class to define and solve an optimization problem for task parameters
    %   This class uses CasADi to optimize task parameters of a robot. It includes stick parameters, 
    %   sphere parameters, and robot configurations. The class also handles robot joint limits and 
    %   defines parameters related to sphere gridpoint spacing and robot DOF.
    
    properties
        opti; % Casadi Opti object for optimization
        
        % Stick parameters
        lstick1;          % Stick 1 length (Casadi variable)
        lstick2;          % Stick 2 length (Casadi variable)
        qstick1;          % Stick 1 rotation angle (Casadi variable)
        qstick2;          % Stick 2 rotation angle (Casadi variable)
        
        lstick1_val;      % Numerical value for Stick 1 length
        lstick2_val;      % Numerical value for Stick 2 length
        qstick1_val;      % Numerical value for Stick 1 rotation angle
        qstick2_val;      % Numerical value for Stick 2 rotation angle
        
        % Sphere parameters
        sphereAngularGridSpacing_val; % Numerical value for sphere's angular gridpoint spacing
                
        sphereRadius;     % Sphere radius (Casadi variable)
        sphereCenter;     % 3D coordinates for the sphere's center (Casadi variable)
        gridPoints;        % Casadi variable for the sphere grid points (depends on sphereCenter and sphereRadius)

        sphereRadius_val; % Numerical value for Sphere radius
        sphereCenter_val; % Numerical 3D coordinates for the sphere's center        
        gridPoints_val;   % Numerical value for the sphere grid points
        
        % Robot configurations
        N; % Number of robot configurations, dependent on sphereAngularGridSpacing_val
        n; % Number of degrees of freedom (DOF) for the robot
        robotConfig;   % Casadi variable for robot configurations, sized (n x N)
        robotConfig_val; % Numerical values for robot configurations
        
        % Forward kinematics properties
        rot;          % Casadi expression for rotation at each configuration
        pos;          % Casadi expression for position at each configuration
        rot_val;          % Numerical values for rotation at each configuration
        pos_val;          % Numerical values for position at each configuration

        % Forward kinematics properties for the stick
        stickPos;     % Casadi expression for stick positions
        stickRot;     % Casadi expression for stick rotations
        stickPos_val;     % Numerical values for stick positions
        stickRot_val;     % Numerical values for stick rotations
        
        % Distances
        dStick2Gridpoints;      % Distance between the stick's end and the grid points
        dRob2SphereCenter;      % Distance between each robot link position and the center of the sphere
        dStick2SphereCenter;    % Distance between each stick position and the center of the sphere
        cost;                    % Cost for the optimization problem
        constraints;             % Constraints for the optimization problem
        
        % Properties for box constraints
        sphereCenterLowerLimit_val; % Numerical value
        sphereCenterUpperLimit_val; % Numerical value
        sphereRadiusLowerLimit_val; % Numerical value
        sphereRadiusUpperLimit_val; % Numerical value
        stickParamsLowerLimit_val;  % Numerical value
        stickParamsUpperLimit_val;  % Numerical value
    
        % Collision sphere and joint limits
        collisionSphereRadius_val; % Numerical value for collision sphere's radius
        jointLimits_val; % Numerical values for robot joint limits
    end
    
    methods
        
        % Constructor
        function obj = taskParameterOptimizer(initValues)
            % Constructor for taskParameterOptimizer class
            % 
            % obj = taskParameterOptimizer(initValues)
            % 
            % INPUT:
            %   initValues: A structure with fields corresponding to the numerical properties.
            %               Fields include: n, lstick1_val, lstick2_val, qstick1_val, qstick2_val, 
            %               sphereRadius_val, sphereCenter_val, sphereAngularGridSpacing_val, 
            %               robotConfig_val, collisionSphereRadius_val, jointLimits_val
            %
            % Validate the presence of required fields in the input structure
            requiredFields = {'n',...
                              'lstick1_val', 'lstick2_val', 'qstick1_val', 'qstick2_val', ...
                              'sphereRadius_val', 'sphereCenter_val', 'sphereAngularGridSpacing_val', ...
                              'robotConfig_val', 'collisionSphereRadius_val', 'jointLimits_val'};
                          
            for field = requiredFields
                if ~isfield(initValues, field{1})
                    error(['Missing required field: ', field{1}]);
                end
            end
            
            % Casadi Opti object
            obj.opti = casadi.Opti();
            
            % Load initial numerical values
            obj = obj.loadNumerical(initValues);
            
            % Create Casadi variables for stick parameters
            obj.lstick1 = obj.opti.variable(1, 1);
            obj.lstick2 = obj.opti.variable(1, 1);
            obj.qstick1 = obj.opti.variable(1, 1);
            obj.qstick2 = obj.opti.variable(1, 1);
            
            % Create Casadi variables for sphere parameters
            obj.sphereRadius = obj.opti.variable(1, 1);
            obj.sphereCenter = obj.opti.variable(3, 1);
            
            % Calculate gridPoints as sum of sphereCenter and gridPoints_val
            obj.gridPoints = obj.sphereCenter + obj.gridPoints_val;
            
            % Create Casadi variable for robot configurations
            obj.robotConfig = obj.opti.variable(obj.n, obj.N);
        end
        
        function obj = loadNumerical(obj, initValues)
            % loadNumerical - Load numerical values for optimization
            %
            % SYNTAX:
            % obj = obj.loadNumerical(initValues)
            %
            % DESCRIPTION:
            % This method loads the numerical values provided in the initValues
            % structure into the corresponding properties of the taskParameterOptimizer
            % class. The initValues structure should contain fields that match the
            % class's properties with the "_val" suffix, as well as 'n' and 'N'.
            %
            % INPUT:
            % initValues: A structure with fields corresponding to the numerical properties.
            %             Fields include: lstick1_val, lstick2_val, qstick1_val, qstick2_val,
            %                             sphereRadius_val, sphereCenter_val, gridPoints_val,
            %                             robotConfig_val, collisionSphereRadius_val, jointLimits_val,
            %                             n, N, ... and other _val properties as per the properties of the class.

            % Validate the presence of required fields in the input structure
            requiredFields = {'n', 'N', ...
                              'lstick1_val', 'lstick2_val', 'qstick1_val', 'qstick2_val', ...
                              'sphereRadius_val', 'sphereCenter_val', 'gridPoints_val', ...
                              'robotConfig_val', 'collisionSphereRadius_val', 'jointLimits_val', ...
                              'sphereCenterLowerLimit_val', 'sphereCenterUpperLimit_val', ...
                              'sphereRadiusLowerLimit_val', 'sphereRadiusUpperLimit_val', ...
                              'stickParamsLowerLimit_val', 'stickParamsUpperLimit_val'};

            for field = requiredFields
                if ~isfield(initValues, field{1})
                    error(['The required field ', field{1}, ' is missing from the input structure.']);
                end
                obj.(field{1}) = initValues.(field{1});
            end
        end
        
        function obj = initialize(obj)
            %initialize - Initialize the casadi.Opti variables with their loaded numerical values.
            %   
            %   USAGE:
            %   obj = obj.initialize()
            %
            %   OUTPUT:
            %   obj: The updated object with casadi.Opti variables initialized.

            % Ensure that numerical values have been loaded
            requiredProps = {'lstick1_val', 'lstick2_val', 'qstick1_val', 'qstick2_val', ...
                             'sphereRadius_val', 'sphereCenter_val', 'robotConfig_val'};

            for propName = requiredProps
                if isempty(obj.(propName{1}))
                    error('Numerical value for %s is empty. Please ensure loadNumerical has been called.', propName{1});
                end
            end

            % Initialize the casadi.Opti variables with the loaded numerical values
            obj.opti.set_initial(obj.lstick1, obj.lstick1_val);
            obj.opti.set_initial(obj.lstick2, obj.lstick2_val);
            obj.opti.set_initial(obj.qstick1, obj.qstick1_val);
            obj.opti.set_initial(obj.qstick2, obj.qstick2_val);
            obj.opti.set_initial(obj.sphereRadius, obj.sphereRadius_val);
            obj.opti.set_initial(obj.sphereCenter, obj.sphereCenter_val);
            obj.opti.set_initial(obj.robotConfig, obj.robotConfig_val);
        end
        
        function obj = forwardKinematics(obj)
            %FORWARDKINEMATICS Propagates each robot and stick configuration through the respective kinematics functions.

            obj.pos_val = cell(1, obj.N);
            obj.rot_val = cell(1, obj.N);
            obj.pos = cell(1, obj.N);
            obj.rot = cell(1, obj.N);

            obj.stickPos_val = cell(1, obj.N);
            obj.stickRot_val = cell(1, obj.N);
            obj.stickPos = cell(1, obj.N);
            obj.stickRot = cell(1, obj.N);

            for k = 1 : obj.N
                % Robot forward kinematics
                [obj.pos_val{k}, obj.rot_val{k}, ~] = pandaAutomaticKinematics(obj.robotConfig_val(:, k));
                [obj.pos{k}, obj.rot{k}, ~] = pandaAutomaticKinematics(obj.robotConfig(:, k));

                % Stick forward kinematics using end-effector's position and orientation
                [obj.stickPos_val{k}, obj.stickRot_val{k}] = stickKinematics(obj.pos_val{k}{end}, obj.rot_val{k}{end}, obj.lstick1_val, obj.lstick2_val, obj.qstick1_val, obj.qstick2_val);
                [obj.stickPos{k}, obj.stickRot{k}] = stickKinematics(obj.pos{k}{end}, obj.rot{k}{end}, obj.lstick1, obj.lstick2, obj.qstick1, obj.qstick2);
            end
        end
        
        function obj = calculateDistances(obj)
            %CALCULATEDISTANCES Compute distances related to the robot, stick, and grid points.

            % Preallocations
            obj.dStick2Gridpoints = casadi.MX.zeros(obj.N, 1);
            obj.dRob2SphereCenter = casadi.MX.zeros(obj.N, length(obj.pos{1}));
            obj.dStick2SphereCenter = casadi.MX.zeros(obj.N, length(obj.stickPos{1}));

            % Calculations for each configuration vector
            for k = 1 : obj.N
                % Square of the distance between the last stick's end position and 
                % the corresponding sphere grid point
                obj.dStick2Gridpoints(k) = sum((obj.stickPos{k}{end} - obj.gridPoints{k}).^2);

                % Square of the distance between each robot link position and 
                % the center of the sphere
                for j = 1 : length(obj.pos{k})
                    obj.dRob2SphereCenter(k, j) = sum((obj.pos{k}{j} - obj.sphereCenter).^2);
                end

                % Square of the distances between each stick position and 
                % the center of the sphere
                for j = 1 : length(obj.stickPos{k})
                    obj.dStick2SphereCenter(k, j) = sum((obj.stickPos{k}{j} - obj.sphereCenter).^2);
                end
            end
        end
    
        function obj = formOptimizationProblem(obj)
            %FORMOPTIMIZATIONPROBLEM Forms the optimization problem using casadi variables.

            % 1. Preallocate constraints

            % Size calculations
            nPos = length(obj.pos{1}); % Assuming length is the same for all pos{k}
            nStick = length(obj.stickPos{1}); % Assuming length is the same for all stickPos{k}
            nStickParams = length(obj.getStickParamVector());

            totalConstraints = 2 * obj.N * (nPos + nStick) + 7 + 2 * nStickParams + 2 * obj.N * obj.n;
            obj.constraints = casadi.MX.zeros(totalConstraints, 1);

            % 2. Call forwardKinematics
            obj = obj.forwardKinematics();

            % 3. Compute distances
            obj = obj.calculateDistances();

            % 4. Calculate the cost and constraints

            % (a) Cost
            obj.cost = sum(obj.dStick2Gridpoints);

            % (b) Collision constraints between robot and sphere
            constraint_idx = 1;
            for k = 1 : obj.N
                for j = 1 : nPos
                    obj.constraints(constraint_idx) = (-obj.dRob2SphereCenter(k, j) + obj.collisionSphereRadius_val.^2);
                    constraint_idx = constraint_idx + 1;
                end
            end

            % (c) Collision constraints between stick and sphere
            for k = 1 : obj.N
                for j = 1 : nStick
                    obj.constraints(constraint_idx) = (-obj.dStick2SphereCenter(k, j) + obj.collisionSphereRadius_val.^2);
                    constraint_idx = constraint_idx + 1;
                end
            end

            % (d) Box constraints for the sphere center and sphere radius
            obj.constraints(constraint_idx : constraint_idx + 2) = -obj.sphereCenter + obj.sphereCenterLowerLimit_val;
            constraint_idx = constraint_idx + 3;

            obj.constraints(constraint_idx : constraint_idx + 2) = obj.sphereCenter - obj.sphereCenterUpperLimit_val;
            constraint_idx = constraint_idx + 3;

            obj.constraints(constraint_idx) = -obj.sphereRadius + obj.sphereRadiusLowerLimit_val;
            constraint_idx = constraint_idx + 1;

            obj.constraints(constraint_idx) = obj.sphereRadius - obj.sphereRadiusUpperLimit_val;
            constraint_idx = constraint_idx + 1;

            % (e) Box constraints for stick parameters
            obj.constraints(constraint_idx : constraint_idx + nStickParams - 1) = -obj.getStickParamVector() + obj.stickParamsLowerLimit_val;
            constraint_idx = constraint_idx + nStickParams;

            obj.constraints(constraint_idx : constraint_idx + nStickParams - 1) = obj.getStickParamVector() - obj.stickParamsUpperLimit_val;
            constraint_idx = constraint_idx + nStickParams;

            % (f) Joint angle box constraints
            jointAngleLimits = casadi.MX.zeros(2 * obj.N * obj.n, 1);
            for k = 1 : obj.N
                jointAngleLimits(2 * (k - 1) * obj.n + 1 : 2 * (k - 1) * obj.n + obj.n) = -obj.robotConfig(:, k) + obj.jointLimits_val(1, :).';
                jointAngleLimits(2 * (k - 1) * obj.n + obj.n + 1 : 2 * k * obj.n) = obj.robotConfig(:, k) - obj.jointLimits_val(2, :).';
            end
            obj.constraints(constraint_idx : constraint_idx + 2 * obj.N * obj.n - 1) = jointAngleLimits;

            % 5. Impose the cost and constraints
            obj.opti.minimize(obj.cost);
            obj.opti.subject_to(obj.constraints <= 0);
        end
        
        function debugInfeasibilities(obj, tol)
            %DEBUGINFEASIBILITIES Evaluates constraints and pinpoints violated ones using a given tolerance.

            % Fetch evaluated constraint values
            g_val = obj.opti.debug.value(obj.constraints);

            % Initialize constraint index tracker
            constraint_idx = 1;

            % 1. Check collision constraints between robot and sphere
            for k = 1 : obj.N
                for j = 1 : length(obj.pos{1})
                    if g_val(constraint_idx) > tol
                        fprintf('Robot-Sphere Collision Constraint Violated at k=%d, j=%d. Value: %f\n', k, j, g_val(constraint_idx));
                    end
                    constraint_idx = constraint_idx + 1;
                end
            end

            % 2. Check collision constraints between stick and sphere
            for k = 1 : obj.N
                for j = 1 : length(obj.stickPos{1})
                    if g_val(constraint_idx) > tol
                        fprintf('Stick-Sphere Collision Constraint Violated at k=%d, j=%d. Value: %f\n', k, j, g_val(constraint_idx));
                    end
                    constraint_idx = constraint_idx + 1;
                end
            end

            % 3. Box constraints for the sphere center and sphere radius
            for i = 1 : 6
                if g_val(constraint_idx) > tol
                    fprintf('Sphere Constraint Violated at Index=%d. Value: %f\n', i, g_val(constraint_idx));
                end
                constraint_idx = constraint_idx + 1;
            end

            % 4. Box constraints for stick parameters
            for i = 1 : 2 * length(obj.getStickParamVector())
                if g_val(constraint_idx) > tol
                    fprintf('Stick Parameter Constraint Violated at Index=%d. Value: %f\n', i, g_val(constraint_idx));
                end
                constraint_idx = constraint_idx + 1;
            end

            % 5. Joint angle box constraints
            for k = 1 : obj.N
                % Lower joint limits
                for j = 1 : obj.n
                    if g_val(constraint_idx) > tol
                        fprintf('Lower Joint Angle Constraint Violated at k=%d, joint=%d. Value: %f\n', k, j, g_val(constraint_idx));
                    end
                    constraint_idx = constraint_idx + 1;
                end
                % Upper joint limits
                for j = 1 : obj.n
                    if g_val(constraint_idx) > tol
                        fprintf('Upper Joint Angle Constraint Violated at k=%d, joint=%d. Value: %f\n', k, j, g_val(constraint_idx));
                    end
                    constraint_idx = constraint_idx + 1;
                end
            end
        end

        
        function stickParams = getStickParamVector(obj)
            %GETSTICKPARAMVECTOR Constructs a vector of stick parameters as CasADi variables.

            stickParams = [
                obj.lstick1;
                obj.lstick2;
                obj.qstick1;
                obj.qstick2;
            ];
        end
        
        function obj = setDefaultIpoptOptions(obj)
            %SETDEFAULTIPOPTOPTIONS Set up default options for the IPOPT solver interfaced by CasADi.

            % Create an IPOPT solver instance
            opts = struct;

            % Here are some common IPOPT options. You can modify, add or remove any as needed.

            % Tolerances
            opts.ipopt.tol = 1e-6; % Desired convergence tolerance.
            opts.ipopt.acceptable_tol = 1e-5; % "Acceptable" convergence tolerance.

            % Iteration limits
            opts.ipopt.max_iter = 3000; % Maximum number of iterations.

            % Derivative options
            opts.ipopt.hessian_approximation = 'limited-memory'; % Use BFGS updates for Hessian approximation

            % Output options
            opts.ipopt.print_level = 5; % Output verbosity level.

            % Any other default settings can be added as required.

            % Set the IPOPT options to the optimization problem
            obj.opti.solver('ipopt', opts);
        end
        
        function plotCurrentGuesses(obj)
            % Extract current values
            pos_val = cellfun(@(cell_elem) cellfun(@(x) obj.opti.debug.value(x), cell_elem, 'UniformOutput', false), obj.pos, 'UniformOutput', false);
            rot_val = cellfun(@(cell_elem) cellfun(@(x) obj.opti.debug.value(x), cell_elem, 'UniformOutput', false), obj.rot, 'UniformOutput', false);
            posSticks_val = cellfun(@(cell_elem) cellfun(@(x) obj.opti.debug.value(x), cell_elem, 'UniformOutput', false), obj.stickPos, 'UniformOutput', false);
            rotSticks_val = cellfun(@(cell_elem) cellfun(@(x) obj.opti.debug.value(x), cell_elem, 'UniformOutput', false), obj.stickRot, 'UniformOutput', false);

            % Set up the figure
            figure('WindowState', 'maximized');
            hold on;
            xlim([-1, 1.5]);
            ylim([-1, 1]);
            zlim([-1, 1.5]);
            view(30, 30);

            for k = 1 : obj.N
                % Clear the previous plot
                cla;

                % Plot the robot
                robotPlot(pos_val{k}, rot_val{k});
                hold on;

                % Plot the stick
                stickPlot(posSticks_val{k}, rotSticks_val{k});
                hold on;

                % Plot the gridpoints
                plot3(obj.gridPoints_val(1, :), obj.gridPoints_val(2, :), obj.gridPoints_val(3, :), 'bo');

                % Pause for 5 seconds
                pause(5);
            end
        end

    end
end
