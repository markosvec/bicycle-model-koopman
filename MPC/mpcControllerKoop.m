function output = MPCControllerKoop(currentX, totalRef, totalSteer, t, T, nSignal, nRef)
% MPC CONTROLLER
%   The function solves the MPC problem and returns control input,
%   diagnostics information and slack variable values. Optimization problem
%   is solved using YALMIP toolbox (https://yalmip.github.io/).

persistent uout sysKoop slipAngMin controller vehicle N

% Initialize persistent variable values
if (t == 0)
    load('koopData.mat');
    vehicle = LoadVehicleParameters();
    
    Akoop = sysKoop.A;
    Bkoop = sysKoop.B;
    Ckoop = sysKoop.C;
    uout = zeros(size(Bkoop,2),1);
    
    nz = size(Akoop,1); % Number of states
    nx = size(Ckoop,1);
    nu = size(Bkoop,2); % Number of inputs
    N = 10; % Prediction horizon
    Nc = 10; % Control horizon

    % Define data for the MPC controller
    % Input and input rate constraints
    slipAngMax = 5/180*pi;
    slipAngMin = -5/180*pi;
    
    lowerInputLim = [-0.035*vehicle.Cxf; slipAngMin*vehicle.Cyf; -0.035*vehicle.Cxr];
    upperInputLim = -lowerInputLim;
    
    lowerInputRateLim = [-0.0075*vehicle.Cxf; slipAngMin/10*vehicle.Cyf;  -0.0075*vehicle.Cxr]/T;
    upperInputRateLim = -lowerInputRateLim;
    
    inputNorm = 1./upperInputLim.^2;
    inputRateNorm = 1./upperInputRateLim.^2;
    
    R = [1 0 0;
         0 1 0; 
         0 0 1].*inputNorm;
    S = [1 0 0;
         0 1 0; 
         0 0 1].*inputRateNorm;
    Q = [1 0 0; 
         0 0 0; 
         0 0 1e6];

    % Avoid explosion of internally defined variables in YALMIP
    yalmip('clear');

    % Setup the optimization problem
    u = sdpvar(repmat(nu,1,N-1), ones(1,N-1));
    z = sdpvar(repmat(nz,1,N), ones(1,N));
    u0 = sdpvar(nu,1);
    r = sdpvar(repmat(nx,1,N), ones(1,N));
    
    constraints = [];
    objective = 0;
    for k = 1:N-1

        if (k == 1)
            pastU = u0;
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

        objective = objective + (Ckoop*z{k}-r{k})'*Q*(Ckoop*z{k}-r{k});
        constraints = [constraints, z{k+1} == Akoop*z{k} + Bkoop*u{k}];
    end
    objective = objective + (Ckoop*z{N}-r{N})'*Q*(Ckoop*z{N}-r{N}); % Terminal cost
    

    % Setup the optimization solver
    ops = sdpsettings('solver','gurobi','verbose',0);
    
    parameters_in = {z{1}, [r{:}], u0};
    solutions_out = {[u{:}], [z{:}]};
    controller = optimizer(constraints, objective,ops, parameters_in, solutions_out);
end

% Extract reference and steering data
[ref, steer] = LTVReference(totalRef, totalSteer, nSignal, nRef, N, T, t);

currentZ = sysKoop.Psi(currentX);
inputs = {currentZ, ref', uout};
[solutions, diagnostics] = controller{inputs};  

% Output
uout = solutions{1}(:,1);
output = [uout; diagnostics];

disp(['Feasible: ',num2str(diagnostics),' Time: ', num2str(t)]);
end