classdef taskParameterOptimizer
    %taskParameterOptimizer - A class to define and solve an optimization problem for task parameters
    %   This class uses CasADi to optimize task parameters of a robot. It includes stick parameters, 
    %   sphere parameters, and robot configurations. The class also handles robot joint limits and 
    %   defines parameters related to sphere gridpoint spacing and robot DOF.
    
    properties
        opti; % Casadi Opti object for optimization
        
        % Stick parameters
        stickParamVars;    % Casadi variable for stick parameters
        lstick1;          % Stick 1 length (Casadi variable)
        lstick2;          % Stick 2 length (Casadi variable)
        qstick1;          % Stick 1 rotation angle (Casadi variable)
        qstick2;          % Stick 2 rotation angle (Casadi variable)
        stickParamValues; % Numerical values for stick parameters
        
        lstick1_val;      % Numerical value for Stick 1 length
        lstick2_val;      % Numerical value for Stick 2 length
        qstick1_val;      % Numerical value for Stick 1 rotation angle
        qstick2_val;      % Numerical value for Stick 2 rotation angle
        
        % Sphere parameters
        sphereParamVars;  % Casadi variable for sphere parameters
        sphereRadius;     % Sphere radius (Casadi variable)
        sphereCenter;     % 3D coordinates for the sphere's center (Casadi variable)
        sphereParamValues;% Numerical values for sphere parameters
        
        sphereRadius_val; % Numerical value for Sphere radius
        sphereCenter_val; % Numerical 3D coordinates for the sphere's center
        sphereAngularGridSpacing_val; % Numerical value for sphere's angular gridpoint spacing
        
        gridPoints_val;        % Numerical value for the sphere grid points
        gridPointsVars;        % Casadi variable for the sphere grid points (depends on sphereCenter and sphereRadius)
        
        % Robot configurations
        N; % Number of robot configurations, dependent on sphereAngularGridSpacing_val
        n; % Number of degrees of freedom (DOF) for the robot
        robotConfigVars;   % Casadi variable for robot configurations, sized (n x N)
        robotConfigValues; % Numerical values for robot configurations
        
        % Forward kinematics properties
        rotVars;          % Casadi expression for rotation at each configuration
        posVars;          % Casadi expression for position at each configuration
        rot_val;          % Numerical values for rotation at each configuration
        pos_val;          % Numerical values for position at each configuration

        % Forward kinematics properties for the stick
        stickPosVars;     % Casadi expression for stick positions
        stickRotVars;     % Casadi expression for stick rotations
        stickPos_val;     % Numerical values for stick positions
        stickRot_val;     % Numerical values for stick rotations
        
        % Distances
        dStick2GridpointsVars;      % Distance between the stick's end and the grid points
        dRob2SphereCenterVars;      % Distance between each robot link position and the center of the sphere
        dStick2SphereCenterVars;    % Distance between each stick position and the center of the sphere
        costVars;                    % Cost for the optimization problem
        constraintsVars;             % Constraints for the optimization problem
        
        % Properties for box constraints
        sphereCenterLowerLimit_val; % Numerical value
        sphereCenterUpperLimit_val; % Numerical value
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
            %               robotConfigValues, collisionSphereRadius_val, jointLimits_val
            %
            % Validate the presence of required fields in the input structure
            requiredFields = {'n',...
                              'lstick1_val', 'lstick2_val', 'qstick1_val', 'qstick2_val', ...
                              'sphereRadius_val', 'sphereCenter_val', 'sphereAngularGridSpacing_val', ...
                              'robotConfigValues', 'collisionSphereRadius_val', 'jointLimits_val'};
                          
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
            obj.stickParamVars = [obj.lstick1; obj.lstick2; obj.qstick1; obj.qstick2];
            
            % Create Casadi variables for sphere parameters
            obj.sphereRadius = obj.opti.variable(1, 1);
            obj.sphereCenter = obj.opti.variable(3, 1);
            obj.sphereParamVars = [obj.sphereRadius; obj.sphereCenter];
            
            % Calculate gridPointsVars as sum of sphereCenter and gridPoints_val
            obj.gridPointsVars = obj.sphereCenter + obj.gridPoints_val;
            
            % Create Casadi variable for robot configurations
            obj.robotConfigVars = obj.opti.variable(obj.n, obj.N);
        end
        
        function obj = loadNumerical(obj, initValues)
            %loadNumerical - Load initial numerical values into properties.
            %   
            %   USAGE:
            %   obj = obj.loadNumerical(initValues)
            %
            %   INPUT:
            %   initValues: A structure with fields corresponding to the numerical properties.
            %               Fields include: n, lstick1_val, lstick2_val, qstick1_val, qstick2_val, 
            %               sphereRadius_val, sphereCenter_val, sphereAngularGridSpacing_val, 
            %               robotConfigValues, collisionSphereRadius_val, jointLimits_val
            %
            % Validate the presence of required fields in the input structure
            requiredFields = {'n',...
                              'lstick1_val', 'lstick2_val', 'qstick1_val', 'qstick2_val', ...
                              'sphereRadius_val', 'sphereCenter_val', 'sphereAngularGridSpacing_val', ...
                              'robotConfigValues', 'collisionSphereRadius_val', 'jointLimits_val'};
                          
            for field = requiredFields
                if ~isfield(initValues, field{1})
                    error(['Missing required field: ', field{1}]);
                end
            end

            % Load the numerical values from the provided structure
            obj.n = initValues.n;
            obj.lstick1_val = initValues.lstick1_val;
            obj.lstick2_val = initValues.lstick2_val;
            obj.qstick1_val = initValues.qstick1_val;
            obj.qstick2_val = initValues.qstick2_val;
            obj.sphereRadius_val = initValues.sphereRadius_val;
            obj.sphereCenter_val = initValues.sphereCenter_val;
            obj.sphereAngularGridSpacing_val = initValues.sphereAngularGridSpacing_val;
            obj.robotConfigValues = initValues.robotConfigValues;
            obj.collisionSphereRadius_val = initValues.collisionSphereRadius_val;
            obj.jointLimits_val = initValues.jointLimits_val;
            
            % Compute grid points
            grid_points = createSphereGridpoints(obj.sphereRadius_val, obj.sphereAngularGridSpacing_val, obj.sphereAngularGridSpacing_val);
            
            % Increment grid points by sphere center
            obj.gridPoints_val = grid_points + obj.sphereCenter_val;
            
            % Compute the number of gridpoints N from the size of grid_points
            obj.N = size(grid_points, 2);
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
                             'sphereRadius_val', 'sphereCenter_val', 'robotConfigValues'};

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
            obj.opti.set_initial(obj.robotConfigVars, obj.robotConfigValues);
        end
        
        function obj = forwardKinematics(obj)
            %FORWARDKINEMATICS Propagates each robot and stick configuration through the respective kinematics functions.

            obj.pos_val = cell(1, obj.N);
            obj.rot_val = cell(1, obj.N);
            obj.posVars = cell(1, obj.N);
            obj.rotVars = cell(1, obj.N);

            obj.stickPos_val = cell(1, obj.N);
            obj.stickRot_val = cell(1, obj.N);
            obj.stickPosVars = cell(1, obj.N);
            obj.stickRotVars = cell(1, obj.N);

            for k = 1 : obj.N
                % Robot forward kinematics
                [obj.pos_val{k}, obj.rot_val{k}, ~] = pandaAutomaticKinematics(obj.robotConfigValues(:, k));
                [obj.posVars{k}, obj.rotVars{k}, ~] = pandaAutomaticKinematics(obj.robotConfigVars(:, k));

                % Stick forward kinematics using end-effector's position and orientation
                [obj.stickPos_val{k}, obj.stickRot_val{k}] = stickKinematics(obj.pos_val{k}{end}, obj.rot_val{k}{end}, obj.lstick1_val, obj.lstick2_val, obj.qstick1_val, obj.qstick2_val);
                [obj.stickPosVars{k}, obj.stickRotVars{k}] = stickKinematics(obj.posVars{k}{end}, obj.rotVars{k}{end}, obj.lstick1, obj.lstick2, obj.qstick1, obj.qstick2);
            end
        end
        
        function obj = calculateDistances(obj)
            %CALCULATEDISTANCES Compute distances related to the robot, stick, and grid points.

            % Preallocations
            obj.dStick2GridpointsVars = casadi.MX.zeros(obj.N, 1);
            obj.dRob2SphereCenterVars = casadi.MX.zeros(obj.N, length(obj.posVars{1}));
            obj.dStick2SphereCenterVars = casadi.MX.zeros(obj.N, length(obj.stickPosVars{1}));

            % Calculations for each configuration vector
            for k = 1 : obj.N
                % Square of the distance between the last stick's end position and 
                % the corresponding sphere grid point
                obj.dStick2GridpointsVars(k) = sum((obj.stickPosVars{k}{end} - obj.gridPointsVars{k}).^2);

                % Square of the distance between each robot link position and 
                % the center of the sphere
                for j = 1 : length(obj.posVars{k})
                    obj.dRob2SphereCenterVars(k, j) = sum((obj.posVars{k}{j} - obj.sphereCenter).^2);
                end

                % Square of the distances between each stick position and 
                % the center of the sphere
                for j = 1 : length(obj.stickPosVars{k})
                    obj.dStick2SphereCenterVars(k, j) = sum((obj.stickPosVars{k}{j} - obj.sphereCenter).^2);
                end
            end
        end
    
        function obj = formOptimizationProblem(obj)
            %FORMOPTIMIZATIONPROBLEM Forms the optimization problem using casadi variables.
            
            % Preallocations
            obj.dStick2GridpointsVars = casadi.zeros(obj.N, 1);
            obj.dRob2SphereCenterVars = casadi.zeros(obj.N, length(obj.posVars{1}));
            obj.dStick2SphereCenterVars = casadi.zeros(obj.N, length(obj.stickPosVars{1}));
            totalConstraints = 2*obj.N*(length(obj.posVars{1}) + length(obj.stickPosVars{1}));
            obj.constraintsVars = casadi.zeros(totalConstraints, 1);

            % 1. Call forwardKinematics
            obj = obj.forwardKinematics();

            % 2. Compute distances
            obj = obj.calculateDistances();

            % 3. Calculate the cost and constraints

            % (a) Cost
            obj.costVars = sum(obj.dStick2GridpointsVars);

            % (b) Collision constraints between robot and sphere
            constraint_idx = 1; % to keep track of the current index in the constraint vector
            for k = 1 : obj.N
                for j = 1 : length(obj.posVars{k})
                    obj.constraintsVars(constraint_idx) = (-obj.dRob2SphereCenterVars(k, j) + obj.collisionSphereRadius_val.^2);
                    constraint_idx = constraint_idx + 1;
                end
            end

            % (c) Collision constraints between stick and sphere
            for k = 1 : obj.N
                for j = 1 : length(obj.stickPosVars{k})
                    obj.constraintsVars(constraint_idx) = (-obj.dStick2SphereCenterVars(k, j) + obj.collisionSphereRadius_val.^2);
                    constraint_idx = constraint_idx + 1;
                end
            end

            % 4. Impose the cost and constraints
            obj.opti.minimize(obj.costVars);
            obj.opti.subject_to(obj.constraintsVars <= 0);
        end
        
        function extractSolution(obj)
            % Solution extraction method
            % Assuming solver has been called and opti is populated with a solution:
            obj.stickParamValues = obj.opti.value(obj.stickParamVars);
            obj.sphereParamValues = obj.opti.value(obj.sphereParamVars);
            obj.robotConfigValues = obj.opti.value(obj.robotConfigVars);
        end
    end
end
