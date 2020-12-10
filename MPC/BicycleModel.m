function [dStates] = BicycleModel(state,input)

% Extract states
vx = state(1);
vy = state(2);
yawRate = state(3);

% Extract inputs
deltaF = input(1);
sF = input(2);
sR = input(3);

% Load parameters
vehicle = LoadVehicleParameters();

% Vehicle parameters
a = vehicle.a;
b = vehicle.b;
m = vehicle.Mass;
I = vehicle.Izz;

% Air drag parameters
cw = vehicle.cw;
rho = vehicle.rho;
Aw = vehicle.Aw;

% Tyre parameters
Cxf = vehicle.Cxf;
Cyf = vehicle.Cyf;
Cxr = vehicle.Cxr;
Cyr = vehicle.Cyr;

% Lateral velocities
vyf = vy + a*yawRate;
vyr = vy - b*yawRate;
% Longitudinal velocities
vxf = vx;
vxr = vx;

% TYRE VELOCITIES
% Cornering velocities
vcf = vyf*cos(deltaF) - vxf*sin(deltaF);
vcr = vyr;
% Longitudinal velocities
vlf = vyf*sin(deltaF) + vxf*cos(deltaF);
vlr = vxr;

% TYRE SLIP ANGLES
alpha_f = atan2(vcf,vlf);
alpha_r = atan2(vcr,vlr);

% Longitudinal and cornering forces
Flf = Cxf*sF;
Flr = Cxr*sR;
Fcf = -Cyf*alpha_f;
Fcr = -Cyr*alpha_r;

% AXLE FORCES
% Lateral forces
Fyf = Flf*sin(deltaF) + Fcf*cos(deltaF);
Fyr = Fcr;
% Longitudinal forces
Fxf = Flf*cos(deltaF) - Fcf*sin(deltaF);
Fxr = Flr;

dVx = 1/m * (m*yawRate*vy - 0.5*cw*rho*Aw*sqrt(vx^2+vy^2)*vx + Fxf + Fxr);
dVy = 1/m * (-m*yawRate*vx - 0.5*cw*rho*Aw*sqrt(vx^2+vy^2)*vy + Fyf + Fyr);
dYawRate = 1/I * (Fyf*a - Fyr*b);

dStates = [dVx; dVy; dYawRate];

end

