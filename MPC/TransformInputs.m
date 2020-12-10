function output = TransformInputs(states, inputs, steering0, slip0)
%TRANSFORM INPUTS

% Extract force
forces = inputs(1:3);
Fxr = forces(3);

% Load parameters
vehicle = LoadVehicleParameters();
Cxr = vehicle.Cxr;

fun = @(x) solveEquation(x, forces, states, vehicle);
options = optimoptions('fsolve','Display','off','Algorithm','trust-region');
solution = fsolve(fun, [steering0; slip0], options);

steerAng = solution(1); %steering angle
sF = solution(2); % front wheel slip
sR = Fxr/Cxr; % rear wheel slip

output = [steerAng; sF; sR];

end

function F = solveEquation(x,forces,states, vehicle)

    steer = x(1);
    sF = x(2);

    % Load parameters
    a = vehicle.a; % Distance between the centre of gravity and the front axle
    Cxf = vehicle.Cxf;
    Cyf = vehicle.Cyf;

    % Extract forces
    Fxf = forces(1);
    Fyf = forces(2);

    % Extract states
    vx = states(1);
    vy = states(2);
    yawRate = states(3);

    vyf = vy + a*yawRate;
    vxf = vx;
    
    vcf = vyf*cos(steer) - vxf*sin(steer);
    vlf = vyf*sin(steer) + vxf*cos(steer);
    alphaF = atan2(vcf,vlf);

    F(1) = Cxf*sF*cos(steer) - (-Cyf*alphaF)*sin(steer) - Fxf;
    F(2) = Cxf*sF*sin(steer) + (-Cyf*alphaF)*cos(steer) - Fyf;
end
