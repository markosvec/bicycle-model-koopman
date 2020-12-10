clear; clc;
load dataset.mat;

Ntraj = 5000;
Nsim = 50;
Norder = 12;

% Create random initial states
xLowLim = [10; -30; -10];
xUppLim = [50; 30; 10];
x0 = xLowLim + rand(nx,Ntraj).*(xUppLim - xLowLim);

% Create random input signals
uLow = -50000;
uUpp = 50000;
uTest = uLow + rand(nu,Nsim+1,Ntraj).*(uUpp - uLow);

tspan = linspace(0,Nsim*Ts,Nsim+1);
errVector = zeros(Ntraj,Norder);
sysKoop = cell(Norder,1);
cardinality = zeros(Norder,1);
%%
for k=1:Norder

    % approximate Koopman operator
    order = k;
    b = mpt.basis.poly(nx, order);
    b{end+1} = '1';
    b = unique(b,'stable');
    sysKoop{k} = KoopmanSystem.fitx(x, u, xNext, b, Ts,'linsolve',1);
    cardinality(k) = size(sysKoop{k}.A,1);
end

%% Simulate trajectories
for i=1:Ntraj

    state0 = x0(:,i);
    input = [tspan; uTest(:,:,i)]';

    sim('VehicleSimulationErrorTest'); %simulate nonlinear system
    
    for k=1:Norder
        ksim = sysKoop{k}.simulate(sysKoop{k}.Psi(state0), uTest(:,:,i)); % simulate Koopman system

        errVector(i,k) = 100*sqrt(sum((ksim.Y' - X).^2,'all'))/sqrt(sum(X.^2,'all'));
        disp(strcat(['System order: ', num2str(k), ' Trajectory ', num2str(i),'/',num2str(Ntraj)]));
    end

end

error = mean(error);
