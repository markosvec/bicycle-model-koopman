modelFitEDMD;

%% Simulate the system
x0 = [37.7777; -8.2569; -1.5693];

Nsim = 500;
cutNum = 10;
Tsim = (Nsim-1)*Ts;
sim('VehicleSimulation');

Ncut = Nsim/cutNum;
Ntotal = floor(size(X,1)/Ncut);
x0_koop = x0;
Y_koop_mpt = zeros(size(X'));
Y_lin0 = zeros(size(X'));
Y_lin = zeros(size(X'));
for t=1:Ntotal
    if (t == 1)
        x0_koop = x0; 
        figure;
    else
        x0_koop = X((t-1)*Ncut+1,:)';
    end
    u_koop = U((t-1)*Ncut+1:t*Ncut,:)';
    
    % Simulate Koopman system
    ksim = sysKoopMPT.simulate(sysKoopMPT.Psi(x0_koop),u_koop);
    Y_koop_mpt(:,(t-1)*Ncut+1:t*Ncut) = ksim.Y(:,1:end);
    
    % Simulate lin0 system
    
    % Create linearized approximation
    if (t==1)
        [Ad, Bd] = LinearizedMatrices(x0_koop, u_koop(:,1), Ts);
        sysLin0 = LTISystem('A', Ad, 'B', Bd, 'C', eye(3), 'Ts', Ts);
    end
    lin0sim = sysLin0.simulate(x0_koop,u_koop);
    Y_lin0(:,(t-1)*Ncut+1:t*Ncut) = lin0sim.Y(:,1:end);
    
    % Simulate linx0 system
    [Ad, Bd] = LinearizedMatrices(x0_koop,u_koop(:,1), Ts);
    sysLin = LTISystem('A', Ad, 'B', Bd, 'C', eye(3), 'Ts', Ts);
    linsim = sysLin.simulate(x0_koop,u_koop);
    Y_lin(:,(t-1)*Ncut+1:t*Ncut) = linsim.Y(:,1:end);
    
    % Plot the prediction
    currTime = tout((t-1)*Ncut+1:t*Ncut);
    s1 = subplot(3,1,1);
        plot(currTime,X((t-1)*Ncut+1:t*Ncut,1)','b'); 
        hold on; grid on;
        plot(currTime,Y_koop_mpt(1,(t-1)*Ncut+1:t*Ncut),'r');
        plot(currTime,Y_lin0(1,(t-1)*Ncut+1:t*Ncut),'g');
        plot(currTime,Y_lin(1,(t-1)*Ncut+1:t*Ncut),'c');
    s2 = subplot(3,1,2);
        plot(currTime,X((t-1)*Ncut+1:t*Ncut,2)','b');
        hold on;  grid on;
        plot(currTime,Y_koop_mpt(2,(t-1)*Ncut+1:t*Ncut),'r');
        plot(currTime,Y_lin0(2,(t-1)*Ncut+1:t*Ncut),'g');
        plot(currTime,Y_lin(2,(t-1)*Ncut+1:t*Ncut),'c');
    s3 = subplot(3,1,3);
        plot(currTime,X((t-1)*Ncut+1:t*Ncut,3)','b');
        hold on; grid on;
        plot(currTime,Y_koop_mpt(3,(t-1)*Ncut+1:t*Ncut),'r');
        plot(currTime,Y_lin0(3,(t-1)*Ncut+1:t*Ncut),'g');
        plot(currTime,Y_lin(3,(t-1)*Ncut+1:t*Ncut),'c');
        
    % Save the data
    WriteToFile(strcat(['./data/original',num2str(t),'.dat']), currTime, X((t-1)*Ncut+1:t*Ncut,:));
    WriteToFile(strcat(['./data/koopman',num2str(t),'.dat']), currTime, Y_koop_mpt(:,(t-1)*Ncut+1:t*Ncut)');
    WriteToFile(strcat(['./data/lin',num2str(t),'.dat']), currTime, Y_lin(:,(t-1)*Ncut+1:t*Ncut)');
    WriteToFile(strcat(['./data/linInit',num2str(t),'.dat']), currTime, Y_lin0(:,(t-1)*Ncut+1:t*Ncut)');
end 

set([s1, s2, s3],'xtick',0:Ncut*Ts:Ntotal*Ncut*Ts);
ylabel(s1,'v_x(m/s)');
ylabel(s2,'v_y(m/s)');
xlabel(s3,'t(s)');
ylabel(s3,'\theta(rad/s)');
legend(s1,'original','koopman_{mpt}','linearized at 0', 'linearized at x0');

% Signal error calculation
koopMptError = 100*sqrt(sum((Y_koop_mpt' - X).^2,'all'))/sqrt(sum(X.^2,'all'))
lin0Error = 100*sqrt(sum((Y_lin0' - X).^2,'all'))/sqrt(sum(X.^2,'all'))
linError = 100*sqrt(sum((Y_lin' - X).^2,'all'))/sqrt(sum(X.^2,'all'))