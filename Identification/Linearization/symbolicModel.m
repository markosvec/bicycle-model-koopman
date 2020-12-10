clear; clc;

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

% Extract states
vx = sym('vx','real');
vy = sym('vy','real');
yawRate = sym('yawRate','real');

% Extract inputs
Fxf = sym('Fxf','real');
Fyf = sym('Fyf','real');
Fxr = sym('Fxr','real');

% Lateral velocities
vyf = vy + a*yawRate;
vyr = vy - b*yawRate;
% Longitudinal velocities
vxf = vx;
vxr = vx;

% TYRE VELOCITIES
% Cornering velocities
%vcf = vyf*cos(deltaF) - vxf*sin(deltaF);
vcr = vyr;
% Longitudinal velocities
%vlf = vyf*sin(deltaF) + vxf*cos(deltaF);
vlr = vxr;

% TYRE SLIP ANGLES
%alpha_f = atan2(vcf,vlf);
alpha_r = atan2(vcr,vlr);

% Longitudinal and cornering forces
%Flf = Cxf*sF;
%Flr = Cxr*sR;
%Fcf = -Cyf*alpha_f;
Fcr = -Cyr*alpha_r;

% AXLE FORCES
% Lateral forces
%Fyf = Flf*sin(deltaF) + Fcf*cos(deltaF);
Fyr = Fcr;
% Longitudinal forces
%Fxf = Flf*cos(deltaF) - Fcf*sin(deltaF);
%Fxr = Flr;

dVx = yawRate*vy + 1/m*(-0.5*cw*rho*Aw*sqrt(vx^2+vy^2)*vx + Fxf + Fxr);
dVy = -yawRate*vx + 1/m*(-0.5*cw*rho*Aw*sqrt(vx^2+vy^2)*vy + Fyf + Fyr);
dYawRate = 1/I * (Fyf*a - Fyr*b);

dStates = [dVx; dVy; dYawRate];
states = [vx; vy; yawRate];
input = [Fxf; Fyf; Fxr];

HardcodeLinearization(states, input, dStates);


