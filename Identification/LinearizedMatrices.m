function [Ad, Bd] = LinearizedMatrices(states,input, T)
% LINEARIZED MATRICES
%   The function returns linearized model of a vehicle.

% Initialize state variables
vx = states(1);
vy = states(2);
yawRate = states(3);

% Initialize input variables
Fxf = input(1);
Fyf = input(2);
Fxr = input(3);

if (vx >= 0)
    vx = max(vx, 1e-6);
else
    vx = min(vx, -1e-6);
end

if (vy >= 0)
    vy = max(vy, 1e-6);
else
    vy = min(vy, -1e-6);
end

% Initialize matrices
nx = length(states);
nu = length(input);
A = zeros(nx,nx);
B = zeros(nx,nu);

% Define A matrix elements
% ROW 1
A(1,1) = - (341*(vx^2 + vy^2)^(1/2))/1460000 - (341*vx^2)/(1460000*(vx^2 + vy^2)^(1/2));
A(1,2) = yawRate - (341*vx*vy)/(1460000*(vx^2 + vy^2)^(1/2));
A(1,3) = vy;
% ROW 2
A(2,1) = (6436*(vy - (131*yawRate)/100))/(219*((vy - (131*yawRate)/100)^2 + vx^2)) - yawRate - (341*vx*vy)/(1460000*(vx^2 + vy^2)^(1/2));
A(2,2) = - (341*(vx^2 + vy^2)^(1/2))/1460000 - (6436*vx)/(219*((vy - (131*yawRate)/100)^2 + vx^2)) - (341*vy^2)/(1460000*(vx^2 + vy^2)^(1/2));
A(2,3) = (210779*vx)/(5475*((vy - (131*yawRate)/100)^2 + vx^2)) - vx;
% ROW 3
A(3,1) = -(843116*(vy - (131*yawRate)/100))/(28575*((vy - (131*yawRate)/100)^2 + vx^2));
A(3,2) = (843116*vx)/(28575*((vy - (131*yawRate)/100)^2 + vx^2));
A(3,3) = -(27612049*vx)/(714375*((vy - (131*yawRate)/100)^2 + vx^2));

% Define B matrix elements
% ROW 1
B(1,1) = 1/1752;
B(1,2) = 0;
B(1,3) = 1/1752;
% ROW 2
B(2,1) = 0;
B(2,2) = 1/1752;
B(2,3) = 0;
% ROW 3
B(3,1) = 0;
B(3,2) = 287/457200;
B(3,3) = 0;

% Create discrete system
sysCont = ss(A, B, eye(3), zeros(3,3));
sysDisc = c2d(sysCont, T, 'tustin');
Ad = sysDisc.A;
Bd = sysDisc.B;

end