function yd = ODEFUN(t, y, u_dis, par, data)
% y = [theta_1; theta_2; theta_1d; theta_2d; T];

% Interpolate u
u = interp1(linspace(0,5,data.nDisControls), u_dis, t, data.interpMethod);

P1 = par.m1*par.c1^2 + par.m2*par.l1^2 + par.I1;
P2 = par.m2*par.c2^2 + par.I2;
P3 = par.m2*par.l1*par.c2;

M = [P1 + P2 + 2*P3*cos(y(2)), P2 + P3*cos(y(2));
    P2 + P3*cos(y(2)), P2];

C = [par.b1 - P3*y(4)*sin(y(2)), -P3*(y(3)+y(4))*sin(y(2));
    P3*y(3)*sin(y(2)), par.b2];

G = [-(par.m1*par.c1 + par.m2*par.l1)*par.g*sin(y(1)) - par.m2*par.c2*par.g*sin(y(1)+y(2));
    -par.m2*par.c2*par.g*sin(y(1) + y(2))];

theta_dd = M \ ([y(5); 0] - C*[y(3); y(4)] - G);

Td = par.tau \ (par.km*u - y(5));

yd = [y(3); y(4); theta_dd; Td];

end

