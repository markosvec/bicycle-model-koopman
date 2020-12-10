clear; clc;

x0 = [80/3.6; 0; -5/180*pi];
Tsim = 10;
Ts = 0.01;
vehicle = LoadVehicleParameters();

% Initialize reference parameters
Tref = 4;
fref = 0.7;
Aref = 8;
Tdref = 0.5;

Np = 10; %prediction step number WATCH OUT FOR THIS!!!
sim('RefGen');
[nSignal,nRef] = size(reference.signals.values);

sim('ControlLoopSim');

%% Plot states
figure; subplot(3,1,1); 
plot(states.time, 3.6*states.signals.values(:,1)); hold;
plot(reference.time, 3.6*reference.signals.values(:,1), '--r');
grid; xlabel('t(s)'); ylabel('v_x(km/h)'); title('States');  axis([0 Tsim -inf inf]);
subplot(3,1,2);
plot(states.time, 3.6*states.signals.values(:,2)); hold;
plot(reference.time, 3.6*reference.signals.values(:,2), '--r');
grid; xlabel('t(s)'); ylabel('v_y(km/h)');  axis([0 Tsim -inf inf]);
subplot(3,1,3);
plot(states.time, 180/pi*states.signals.values(:,3)); hold;
plot(reference.time, 180/pi*reference.signals.values(:,3), '--r');
grid; xlabel('t(s)'); ylabel('\theta(°)');  axis([0 Tsim -inf inf]);

% Plot input and diagnostics
figure;
subplot(3,1,1);
plot(input.time, 180/pi*input.signals.values(:,1),'r');
grid; xlabel('t(s)'); ylabel('delta_{f}(deg)');
subplot(3,1,2);
plot(input.time, 100*input.signals.values(:,2),'r');
grid; xlabel('t(s)'); ylabel('s_{f}(%)');
subplot(3,1,3);
plot(input.time, 100*input.signals.values(:,3),'r');
grid; xlabel('t(s)'); ylabel('s_{r}(%)');