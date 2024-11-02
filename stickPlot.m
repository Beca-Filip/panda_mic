function stickPlot(positions, rotations)
% stickPlot - Plots the structure of the sticks and their frame axes based on 
% the provided positions and rotations.
%
% INPUT:
%   positions - Cell array of positions for the end effector and ends of sticks.
%   rotations - Cell array of rotation matrices for each stick and end effector.
%
% OUTPUT:
%   A figure showing the stick structure and orientation.

    % Convert cell arrays to matrix form for easier manipulation
    PosMat = cell2mat(positions);
    RotMat = cell2mat(rotations);

    % Extract rotation components for X, Y, and Z axes
    Ux = RotMat(1, 1:3:end-2);
    Vx = RotMat(2, 1:3:end-2);
    Wx = RotMat(3, 1:3:end-2);
    
    Uy = RotMat(1, 2:3:end-1);
    Vy = RotMat(2, 2:3:end-1);
    Wy = RotMat(3, 2:3:end-1);
    
    Uz = RotMat(1, 3:3:end);
    Vz = RotMat(2, 3:3:end);
    Wz = RotMat(3, 3:3:end);

    % Begin plotting
    hold on;
    
    % Plot stick structure using a unique color
    plot3(PosMat(1, :), PosMat(2, :), PosMat(3, :), 'o-', 'LineWidth', 1.5, 'Color', [0.9290 0.6940 0.1250], 'DisplayName', 'Stick');
    
    % Plot frame axes for each stick position
    quiver3(PosMat(1, :), PosMat(2, :), PosMat(3, :), Ux, Vx, Wx, 'Color', 'r', 'LineWidth', 2, 'AutoScaleFactor', 0.1, 'DisplayName', 'StickFrames X-axes');
    quiver3(PosMat(1, :), PosMat(2, :), PosMat(3, :), Uy, Vy, Wy, 'Color', 'g', 'LineWidth', 2, 'AutoScaleFactor', 0.1, 'DisplayName', 'StickFrames Y-axes');
    quiver3(PosMat(1, :), PosMat(2, :), PosMat(3, :), Uz, Vz, Wz, 'Color', 'b', 'LineWidth', 2, 'AutoScaleFactor', 0.1, 'DisplayName', 'StickFrames Z-axes');

    % Annotate stick numbers (starting from the second position)
    for ii = 2 : size(PosMat, 2)
        text(PosMat(1, ii)+0.005, PosMat(2, ii), PosMat(3, ii), sprintf("$%d$", ii-1), ...
             'Interpreter', 'latex', 'FontSize', 15, 'Color', [0.9290 0.6940 0.1250]);
    end
    
    % Set plot properties for visualization
    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');

    hold off;
end
