function stateUpdate = BicycleModelDiscrete(states, input, Ts)
% BICYCLE MODEL DISCRETE
%   The function accepts current system states and inputs, and calculates 
%   updated discrete system states using ODE45 method.
    
    tspan = [0 Ts];
    [T, Y] = ode45(@BicycleModelODE, tspan, [states', input']);
    stateUpdate = Y(end,1:3)';

end

function derivatives = BicycleModelODE(t, in)

    states = in(1:3);
    input = in(4:6);
    [derivatives] = BicycleModel(states, input);
    derivatives = [derivatives; input];

end


