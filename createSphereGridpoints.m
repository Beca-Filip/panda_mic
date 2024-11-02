function sphereGridpoints = createSphereGridpoints(radius, stepPhiDeg, stepThetaDeg)
%CREATESPHEREGRIDPOINTS Generates a list of grid points on a sphere's surface.
%
% INPUT:
% radius: The radius of the sphere.
% stepPhiDeg: The angular step size in degrees for the azimuth angle (0 to 360).
% stepThetaDeg: The angular step size in degrees for the polar/elevation angle (-90 to 90).
%
% OUTPUT:
% sphereGridpoints: A 3xN matrix where each column represents a point (x, y, z) on the sphere.

% Convert degree steps to radian values for azimuth and polar angles
phiList = deg2rad(0 : stepPhiDeg : 360);
thetaList = deg2rad(-90 : stepThetaDeg : 90);

numPhi = length(phiList);
numTheta = length(thetaList);

% Anonymous function for creating grid points on the sphere's surface
computeGridpoint = @(r, phi, theta) r * [cos(theta) * [cos(phi); sin(phi)]; sin(theta)];

% Initialize matrix to store sphere grid points
sphereGridpoints = nan(3, numPhi * (numTheta - 2) + 2);

% First point is the south pole
sphereGridpoints(:, 1) = computeGridpoint(radius, phiList(1), thetaList(1));

% Compute points for the rest of the sphere excluding the poles
for k = 1 : numTheta - 2
    for l = 1 : numPhi
        idx = 1 + (k-1) * numPhi + l;
        sphereGridpoints(:, idx) = computeGridpoint(radius, phiList(l), thetaList(1 + k));
    end
end

% Last point is the north pole
sphereGridpoints(:, numPhi * (numTheta - 2) + 2) = computeGridpoint(radius, phiList(end), thetaList(end));

end
