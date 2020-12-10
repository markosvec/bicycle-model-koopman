function output = MPCController(currentX, totalRef, totalSteer, t, T, nSignal, nRef)
% MPC CONTROLLER
%   The function solves the MPC problem and returns control input,
%   diagnostics information and slack variable values. Optimization problem
%   is solved using YALMIP toolbox (https://yalmip.github.io/).

persistent oldU oldX uout

nx = 3; % Number of states
nu = 3; % Number of inputs
nSlipAng = 4;
N = 10; % Prediction horizon
Nc = 10; % Control horizon

vehicle = LoadVehicleParameters();

% Extract reference and steering data
[ref, steer] = LTVReference(totalRef, totalSteer, nSignal, nRef, N, T, t);

% Define data for the MPC controller
% Input and input rate constraints
slipAngMax = 5/180*pi;
slipAngMin = -5/180*pi;

% Define data for the MPC controller
% Input and input rate constraints
lowerInputLim = [-45/180*pi; -0.035; -0.035];
upperInputLim = -lowerInputLim;
lowerInputRateLim = [-2/180*pi; -0.0075; -0.0075]/T;
upperInputRateLim = -lowerInputRateLim;

inputNorm = 1./upperInputLim.^2;
inputRateNorm = 1./upperInputRateLim.^2;

R = [1 0 0;
     0 1 0; 
     0 0 1].*inputNorm;
S = [1 0 0;
     0 1 0; 
     0 0 1].*inputRateNorm;
Q = [1e5 0 0; 
     0 0 0; 
     0 0 1];
p = 1e4; 
 
% Avoid explosion of internally defined variables in YALMIP
yalmip('clear');

% Setup the optimization problem
u = sdpvar(repmat(nu,1,N-1), ones(1,N-1));
x = sdpvar(repmat(nx,1,N), ones(1,N));
eps = sdpvar(repmat(nSlipAng,1,N-1), ones(1,N-1));

% Initialize persistent variable values
if (t == 0)
    [oldU{1,1:N-1}] = deal(zeros(nu,1));
    [oldX{1,1:N-1}] = deal(currentX);
    uout = zeros(nu,1);
end

oldX{1} = currentX; % Assign newest value for the current moment
[At,Bt] = LinearizedMatricesCell(oldX, oldU, T); % Calculate linearized system dynamics
delta = DeltaSignal(oldX, oldU, At, Bt, N-1, T); % Calculate delta signal
G = SlipAngleConstraints(steer(2:end), vehicle, slipAngMax, slipAngMin);

constraints = [];
objective = 0;
for k = 1:N-1
    
    if (k == 1)
        pastU = uout;
    else
        pastU = u{k-1};
    end
    
    if (k < Nc)
        objective = objective + u{k}'*S*u{k} + ((u{k}-pastU)/T)'*R*((u{k}-pastU)/T);
        constraints = [constraints, lowerInputLim <= u{k} <= upperInputLim];
        constraints = [constraints, lowerInputRateLim <= (u{k}-pastU)/T <= upperInputRateLim];
    else
        constraints = [constraints, u{k} == pastU]; % Input blocking
    end
    
    objective = objective + (x{k}-ref(k,:)')'*Q*(x{k}-ref(k,:)') + p*eps{k}'*eps{k};
    constraints = [constraints, x{k+1} == At{k}*x{k} + Bt{k}*u{k} + delta{k}];
    constraints = [constraints, G(:,(k-1)*nx+1:k*nx)*x{k+1} <= eps{k}, eps{k} >= 0];
    
    constraints = [constraints, u{k}(1) == steer(k)]; % steering reference
end
objective = objective + (x{N}-ref(N,:)')'*Q*(x{N}-ref(N,:)'); % Terminal cost

constraints = [constraints, x{1} == currentX];

% Setup the optimization solver
ops = sdpsettings('solver','osqp','verbose',0);
ops.osqp.max_iter = 100000;
ops.osqp.eps_abs = 1e-6;
ops.osqp.eps_rel = 1e-6;

solution = optimize(constraints,objective,ops);

% Output
uout = value(u{1});
output = [uout; solution.problem];

% Save persistent variables
for i = 1:N-1
    if(i == N-1)
        oldU{i} = value(u{i});
    else
        oldU{i} = value(u{i+1});
    end
    oldX{i} = value(x{i+1}); 
end

disp(['Feasible: ',num2str(solution.problem),' Time: ', num2str(t)]);
end