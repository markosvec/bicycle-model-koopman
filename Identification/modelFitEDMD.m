clear; clc;

if(exist('dataset.mat', 'file') ~= 2) 

    N = 20;
    Ts = 0.01;
    nx = 3;
    nu = 3;
    nD = nx + nu;
    
    vx = linspace(10,50,N);
    vy = linspace(-30,30,N);
    dTheta = linspace(-10,10,N);
    forceOne = linspace(-5,5,N);
    forceTwo = linspace(-50,50,N);
    forceThree = linspace(-500,500,N);
    forceFour = linspace(-5000,5000,N);

    D1 = zeros(nD,N^4);
    D2 = zeros(nD,N^4);
    D3 = zeros(nD,N^4);
    D4 = zeros(nD,N^4);
    cntr = 1;

    % Create all possible states
    for a=1:N
        for b=1:N
            for c=1:N
                for d=1:N
                    D1(:,cntr) = [vx(a); vy(b); dTheta(c); 
                                forceOne(d); forceTwo(d); forceThree(d)];
                    D2(:,cntr) = [vx(a); vy(b); dTheta(c); 
                                forceFour(d); forceOne(d); forceTwo(d)];
                    D3(:,cntr) = [vx(a); vy(b); dTheta(c); 
                                forceThree(d); forceFour(d); forceOne(d)];
                    D4(:,cntr) = [vx(a); vy(b); dTheta(c); 
                                forceTwo(d); forceThree(d); forceFour(d)];
                            
                   cntr = cntr + 1;
                   disp(strcat([num2str(a),' ',num2str(b),' ',num2str(c), ' ', num2str(d)]));
                end
                %{
                D1(:,cntr) = [vx(a); vy(b); dTheta(c); 0; 0; 0];
                cntr = cntr + 1;
                disp(strcat([num2str(a),' ',num2str(b),' ',num2str(c)]));
                %}
            end
        end
    end
    
    dataset = [D1, D2, D3, D4]; % Create dataset matrix
    [~, mD] = size(dataset);
    
    x = dataset(1:nx,:);
    u = dataset(nx+1:end,:);
    xNext = zeros(nx,mD);
    %%
    for i=1:mD
        xNext(:,i) = BicycleModelDiscrete(x(:,i),u(:,i), Ts);
        disp(strcat(['Prediction step: ', num2str(i),'/',num2str(mD)]));
    end
    
    % Clear unnecessary data and save the rest
    clear D1 D2 D3 D4...
        forceOne forceTwo forceThree forceFour...
        cntr vx vy dTheta a b c d i;
    save dataset.mat;

else
    disp('Dataset loaded!');
    load dataset.mat;
end

%% CREATE SYSTEM USING MPT TOOLBOX
% Create Koopman approximation
orderMPT = 4;
b = mpt.basis.poly(nx, orderMPT);
b{end+1} = '1';
b = unique(b,'stable');
sysKoopMPT = KoopmanSystem.fitx(x, u, xNext, b, Ts,'linsolve',1);
