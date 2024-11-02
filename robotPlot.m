function robotPlot(positions, rotations)
% robotPlot - Plots the robot's structure and its frame axes based on the 
% provided positions and rotations of each joint.
%
% INPUT:
%   positions - Cell array of joint positions.
%   rotations - Cell array of rotation matrices for each joint.
%
% OUTPUT:
%   A figure showing the robot structure and orientation of each joint.

    % Convert cell arrays to matrix form
    Pos = [zeros(3, 1), cell2mat(positions)];
    Rot = [eye(3), cell2mat(rotations)];

    % Extract rotation components for X, Y, and Z axes
    Ux = Rot(1, 1:3:end-2);
    Vx = Rot(2, 1:3:end-2);
    Wx = Rot(3, 1:3:end-2);
    
    Uy = Rot(1, 2:3:end-1);
    Vy = Rot(2, 2:3:end-1);
    Wy = Rot(3, 2:3:end-1);
    
    Uz = Rot(1, 3:3:end);
    Vz = Rot(2, 3:3:end);
    Wz = Rot(3, 3:3:end);
    
    % Hold the following plots
    hold on;
    
    % Plot robot structure
    rob = plot3(Pos(1, :), Pos(2, :), Pos(3, :), 'ko-', 'LineWidth', 1.5, 'DisplayName', 'Robot');
    
    % Plot frame axes for each joint
    quiver3(Pos(1, :), Pos(2, :), Pos(3, :), Ux, Vx, Wx, 'Color', 'r', 'LineWidth', 2, 'AutoScaleFactor', 0.1, 'DisplayName', 'RobotFrames X-axes');
    quiver3(Pos(1, :), Pos(2, :), Pos(3, :), Uy, Vy, Wy, 'Color', 'g', 'LineWidth', 2, 'AutoScaleFactor', 0.1, 'DisplayName', 'RobotFrames Y-axes');
    quiver3(Pos(1, :), Pos(2, :), Pos(3, :), Uz, Vz, Wz, 'Color', 'b', 'LineWidth', 2, 'AutoScaleFactor', 0.1, 'DisplayName', 'RobotFrames Z-axes');

    % Annotate joint numbers
    for ii = 1 : size(Pos, 2)
        text(Pos(1, ii)+0.005, Pos(2, ii), Pos(3, ii), sprintf("$%d$", ii-1), 'Interpreter', 'latex', 'FontSize', 15);
    end
    
    % Set plot properties for better visualization
    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Robot Structure and Orientation');

    hold off;
end
