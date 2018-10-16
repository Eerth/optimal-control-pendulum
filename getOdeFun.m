function [h_Fyd, h_Fjac] = getOdeFun(par)

syms t q1 q2 qd1 qd2 tau u real
y = [q1, q2, qd1, qd2, tau]';

% q1 = y(1);
% q2 = y(2);
% qd1 = y(3);
% qd2 = y(4);
% tau = y(5);

P1 = par.m1*par.c1^2 + par.m2*par.l1^2 + par.I1;
P2 = par.m2*par.c2^2 + par.I2;
P3 = par.m2*par.l1*par.c2;

g1 = (par.m1*par.c1 + par.m2*par.l1)*par.g;
g2 = par.m2*par.c2*par.g;

M = [P1 + P2 + 2*P3*cos(q2), P2 + P3*cos(q2);
    P2 + P3*cos(q2), P2];

C = [par.b1 - P3*qd2*sin(q2), -P3*(qd1+qd2)*sin(q2);
    P3*qd1*sin(q2), par.b2];

G = [-g1*sin(q1) - g2*sin(q1+q2);
    -g2*sin(q1 + q2)];

theta_dd = M \ ([tau; 0] - C*[qd1; qd2] - G);

Td = par.te \ (par.km*u - tau);

% Cost
Ld = u^2 + 1;
%Ld = abs(tau * qd1);

% yd
yd = [qd1; qd2; theta_dd; Td; Ld];
h_Fyd = matlabFunction(yd, 'Vars', {t, y, u});

% Jacobian
Fjac = jacobian(yd, y);
h_Fjac = matlabFunction(Fjac, 'Vars', {t, y});

%J_u = jacobian(yd, u);

end