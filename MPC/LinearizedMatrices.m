function [Ad, Bd] = LinearizedMatrices(states,input, T)
% LINEARIZED MATRICES
%   The function returns linearized model of a vehicle.

% Initialize state variables
vx = states(1);
vy = states(2);
yawRate = states(3);

% Initialize input variables
deltaF = input(1);
sF = input(2);
sR = input(3);

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
A(1,1) = - (341*(vx^2 + vy^2)^(1/2))/1460000 - (341*vx^2)/(1460000*(vx^2 + vy^2)^(1/2)) - (6436*sin(deltaF)*(sin(deltaF)/(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200)) - (cos(deltaF)*(vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200)))/(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2)*(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2)/(219*((vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2 + (vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200))^2));
A(1,2) = yawRate - (341*vx*vy)/(1460000*(vx^2 + vy^2)^(1/2)) + (6436*sin(deltaF)*(cos(deltaF)/(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200)) + (sin(deltaF)*(vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200)))/(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2)*(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2)/(219*((vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2 + (vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200))^2));
A(1,3) = vy + (6436*sin(deltaF)*((287*cos(deltaF))/(200*(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))) + (287*sin(deltaF)*(vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200)))/(200*(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2))*(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2)/(219*((vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2 + (vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200))^2));
% ROW 2
A(2,1) = (6436*(vy - (131*yawRate)/100))/(219*((vy - (131*yawRate)/100)^2 + vx^2)) - yawRate - (341*vx*vy)/(1460000*(vx^2 + vy^2)^(1/2)) + (6436*cos(deltaF)*(sin(deltaF)/(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200)) - (cos(deltaF)*(vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200)))/(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2)*(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2)/(219*((vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2 + (vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200))^2));
A(2,2) = - (341*(vx^2 + vy^2)^(1/2))/1460000 - (6436*vx)/(219*((vy - (131*yawRate)/100)^2 + vx^2)) - (341*vy^2)/(1460000*(vx^2 + vy^2)^(1/2)) - (6436*cos(deltaF)*(cos(deltaF)/(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200)) + (sin(deltaF)*(vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200)))/(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2)*(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2)/(219*((vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2 + (vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200))^2));
A(2,3) = (210779*vx)/(5475*((vy - (131*yawRate)/100)^2 + vx^2)) - vx - (6436*cos(deltaF)*((287*cos(deltaF))/(200*(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))) + (287*sin(deltaF)*(vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200)))/(200*(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2))*(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2)/(219*((vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2 + (vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200))^2));
% ROW 3
A(3,1) = (923566*cos(deltaF)*(sin(deltaF)/(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200)) - (cos(deltaF)*(vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200)))/(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2)*(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2)/(28575*((vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2 + (vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200))^2)) - (843116*(vy - (131*yawRate)/100))/(28575*((vy - (131*yawRate)/100)^2 + vx^2));
A(3,2) = (843116*vx)/(28575*((vy - (131*yawRate)/100)^2 + vx^2)) - (923566*cos(deltaF)*(cos(deltaF)/(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200)) + (sin(deltaF)*(vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200)))/(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2)*(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2)/(28575*((vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2 + (vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200))^2));
A(3,3) = - (27612049*vx)/(714375*((vy - (131*yawRate)/100)^2 + vx^2)) - (923566*cos(deltaF)*((287*cos(deltaF))/(200*(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))) + (287*sin(deltaF)*(vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200)))/(200*(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2))*(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2)/(28575*((vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2 + (vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200))^2));

% Define B matrix elements
% ROW 1
B(1,1) = (6436*cos(deltaF)*atan2(cos(deltaF)*(vy + (287*yawRate)/200) - vx*sin(deltaF), vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200)))/219 - (10964*sF*sin(deltaF))/219 - (6436*sin(deltaF)*((vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200))^2/(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2 + 1)*(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2)/(219*((vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2 + (vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200))^2));
B(1,2) = (10964*cos(deltaF))/219;
B(1,3) = 10964/219;
% ROW 2
B(2,1) = (6436*cos(deltaF)*atan2(cos(deltaF)*(vy + (287*yawRate)/200) - vx*sin(deltaF), vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200)))/219 - (10964*sF*sin(deltaF))/219 - (6436*sin(deltaF)*((vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200))^2/(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2 + 1)*(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2)/(219*((vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2 + (vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200))^2));
B(2,2) = (10964*sin(deltaF))/219;
B(2,3) = 0;
% ROW 3
B(3,1) = (1573334*sF*cos(deltaF))/28575 + (923566*atan2(cos(deltaF)*(vy + (287*yawRate)/200) - vx*sin(deltaF), vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))*sin(deltaF))/28575 + (923566*cos(deltaF)*((vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200))^2/(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2 + 1)*(vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2)/(28575*((vx*cos(deltaF) + sin(deltaF)*(vy + (287*yawRate)/200))^2 + (vx*sin(deltaF) - cos(deltaF)*(vy + (287*yawRate)/200))^2));
B(3,2) = (1573334*sin(deltaF))/28575;
B(3,3) = 0;

% Create discrete system
sysCont = ss(A, B, eye(3), zeros(3,3));
sysDisc = c2d(sysCont, T, 'tustin');
Ad = sysDisc.A;
Bd = sysDisc.B;

end