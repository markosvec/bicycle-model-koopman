function [F, angleSym] = TMEasy(sx, sy, Fz, mi, in, dF0, angle)

K = mi.*Fz;
B = pi - asin(in./mi);
A = K.*B./dF0;
G = (A(2, :).*K(1, :).*B(1, :))./(A(1, :).*K(2, :).*B(2, :));

s = [sx; -sy./G];

angleSym = atan2(s(2, :), s(1, :));

f = K.*sin( B .* (1 - exp(-sqrt(s.^2 + 1e-5)./A)));
f_norm = 1/2*(f(1, :) + f(2, :) + (f(1, :) - f(2, :))...
.*cos(2*angle));

F = [ f_norm.*cos(angle); f_norm.*sin(angle) ].*0.5*(1+tanh(10e5*Fz));

end