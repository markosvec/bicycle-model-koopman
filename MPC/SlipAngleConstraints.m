function G = SlipAngleConstraints(steer, vehicle, slipAngMax, slipAngMin)
%SLIP ANGLE CONSTRAINTS
%   Function determines matrix which constraints slip angles

N = length(steer);
a = vehicle.a;
b = vehicle.b;

Gcell = cell(N,1);
for i=1:N
    vcfVect = [-sin(steer(i)) cos(steer(i)) a*cos(steer(i))];
    vlfVect = [cos(steer(i)) sin(steer(i)) a*sin(steer(i))];

    vcrVect = [0 1 -b];
    vlrVect = [1 0 0];

    Gcell{i} = [vcfVect - slipAngMax*vlfVect;
               -vcfVect + slipAngMin*vlfVect;
                vcrVect - slipAngMax*vlrVect;
               -vcrVect + slipAngMin*vlfVect];
end

G = cell2mat(Gcell');

end

