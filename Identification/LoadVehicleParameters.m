function vehicle = LoadVehicleParameters()
% LOAD VEHICLE PARAMETERS
%   The function creates and returns vehicle object based on the specified
%   parameters.

a = 1.435; % Distance between the centre of gravity and the front axle
b = 1.31;  % Distance between the centre of gravity and the rear axle
m = 1752;  % Mass
I = 2286;  % Moment of inertia around z axis
cw = 0.31; % Air drag coefficient
rho = 1.2; % Air density
Aw = 2.2;  % Projected area in a transversal view

Cxf = 87712;
Cxr = 87712;
Cyf = 51488;
Cyr = 51488;

vehicle = Vehicle(m, I, a, b, Cxf, Cxr, Cyf, Cyr, cw, rho, Aw);
end

