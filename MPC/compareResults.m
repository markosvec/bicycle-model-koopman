clear; clc;
load('linSim');
load('koopSim');

%% Plot states
figure; subplot(3,1,1);
plot(states.time, 3.6*states.signals.values(:,1)); hold;
plot(statesKoop.time, 3.6*statesKoop.signals.values(:,1));
plot(reference.time, 3.6*reference.signals.values(:,1), '--r');
grid; xlabel('t(s)'); ylabel('v_x(km/h)'); title('States');
subplot(3,1,2);
plot(states.time, 3.6*states.signals.values(:,2)); hold;
plot(statesKoop.time, 3.6*statesKoop.signals.values(:,2));
plot(reference.time, 3.6*reference.signals.values(:,2), '--r');
grid; xlabel('t(s)'); ylabel('v_y(km/h)');
subplot(3,1,3);
plot(states.time, 180/pi*states.signals.values(:,3)); hold;
plot(statesKoop.time, 180/pi*statesKoop.signals.values(:,3));
plot(reference.time, 180/pi*reference.signals.values(:,3), '--r');
grid; xlabel('t(s)'); ylabel('\theta(°)');

% Plot input and diagnostics
figure;
subplot(3,1,1);
plot(input.time, input.signals.values(:,1)); hold;
plot(inputKoop.time, inputKoop.signals.values(:,1));
grid; xlabel('t(s)'); ylabel('delta_{f}(deg)');
subplot(3,1,2);
plot(input.time, input.signals.values(:,2)); hold;
plot(inputKoop.time, inputKoop.signals.values(:,2));
grid; xlabel('t(s)'); ylabel('s_{f}(%)');
subplot(3,1,3);
plot(input.time, input.signals.values(:,3)); hold;
plot(inputKoop.time, inputKoop.signals.values(:,3));
grid; xlabel('t(s)'); ylabel('s_{r}(%)');