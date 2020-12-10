function d = DeltaSignal(x, u, At, Bt, N, T)
% DELTA SIGNAL 
%   The functions creates and returns delta signal needed
%   to calculate deviations from a steady state.

currX = x{1};
d = cell(N,1);
for i=1:N
    newX = BicycleModelDiscrete(currX, u{i}, T);
    d{i} = newX - At{i}*currX - Bt{i}*u{i};
    currX = newX;
end
end

