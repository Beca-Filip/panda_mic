function plot3DSphere(center, radius, varargin)
    % PLOT3DSPHERE Plots a 3D sphere with the specified center and radius.
    % 
    % Inputs:
    % - center: A 1x3 vector specifying the x, y, z coordinates of the sphere's center.
    % - radius: A scalar value for the sphere's radius.
    % - varargin: Optional argument to specify the size of the meshgrid (default is 20).
    % 
    % Example usage:
    % plot3DSphere([0, 0, 0], 5)
    % plot3DSphere([1, 2, 3], 5, 50)

    % If the optional meshgrid size is specified, use it. Otherwise, default to 20.
    if ~isempty(varargin)
        meshSize = varargin{1};
    else
        meshSize = 20;
    end
    
    % Create the meshgrid
    [x, y, z] = sphere(meshSize);
    
    % Scale and translate the sphere to the desired location
    x = x * radius + center(1);
    y = y * radius + center(2);
    z = z * radius + center(3);
    
    % Plot the sphere
    surf(x, y, z, 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'DisplayName', 'Sphere');
end
