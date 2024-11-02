function points = equidistantSpherePoints(radius, N)
    % N: number of points
    % radius: radius of the sphere
    % points: 3 x N matrix of points on the sphere

    golden_ratio = (1 + sqrt(5)) / 2;
    theta = 2 * pi * golden_ratio; % golden angle

    % Create an array for each point's index
    indices = 0:N-1;

    % Calculate the y and z coordinates
    y = 1 - (indices / (N-1)) * 2; % y goes from 1 to -1
    radius_scale = sqrt(1 - y .* y); % scale factor for x,z

    phi = theta * indices; % angle for each point

    % Convert polar to cartesian coordinates
    x = cos(phi) .* radius_scale;
    z = sin(phi) .* radius_scale;

    % Scale by sphere's radius
    x = x * radius;
    y = y * radius;
    z = z * radius;

    % Return the points
    points = [x; y; z];
end