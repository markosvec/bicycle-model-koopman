classdef Vehicle
    % VEHICLE
    %   Class which contains all relevant vehicle model parameters.
    
    properties
        Mass % Mass
        Izz  % Moment of inertia around z axis
        a    % Distance between the centre of gravity and the front axle
        b    % Distance between the centre of gravity and the rear axle
        Cxf  % Front tyre longitudinal stiffness
        Cxr  % Rear tyre longitudinal stiffness
        Cyf  % Front tyre cornering stiffness
        Cyr  % Rear tyre cornering stiffness
        cw   % Air drag coefficient
        rho  % Air density
        Aw   % Projected area in a transversal view
    end
    
    methods
        function obj = Vehicle(m, I, a, b, Cxf, Cxr, Cyf, Cyr, cw, rho, Aw)
           % Create object
           obj.Mass = m;
           obj.Izz = I;
           obj.a = a;
           obj.b = b;
           obj.Cxf = Cxf;
           obj.Cxr = Cxr;
           obj.Cyf = Cyf;
           obj.Cyr = Cyr;
           obj.cw = cw;
           obj.rho = rho;
           obj.Aw = Aw;
        end
    end
end

