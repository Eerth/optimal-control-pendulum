function yd = ODEFUN(t, y, u_dis, T, par, data)
% y = [theta_1; theta_2; theta_1d; theta_2d; T];

% Interpolate u
u = interp1(linspace(0,T,data.nDisControls), u_dis, t, data.interpMethod);

% q1 = y(1);
% q2 = y(2);
% qd1 = y(3);
% qd2 = y(4);
% tau = y(5);
% 
% P1 = par.m1*par.c1^2 + par.m2*par.l1^2 + par.I1;
% P2 = par.m2*par.c2^2 + par.I2;
% P3 = par.m2*par.l1*par.c2;
% 
% g1 = (par.m1*par.c1 + par.m2*par.l1)*par.g;
% g2 = par.m2*par.c2*par.g;
% 
% M = [P1 + P2 + 2*P3*cos(q2), P2 + P3*cos(q2);
%     P2 + P3*cos(q2), P2];
% 
% C = [par.b1 - P3*qd2*sin(q2), -P3*(qd1+qd2)*sin(q2);
%     P3*qd1*sin(q2), par.b2];
% 
% G = [-g1*sin(q1) - g2*sin(q1+q2);
%     -g2*sin(q1 + q2)];
% 
% theta_dd = M \ ([tau; 0] - C*[qd1; qd2] - G);
% 
% Td = par.te \ (par.km*u - tau);
% 
% % Cost
% Ld = u^2 + 1;
% %Ld = abs(tau * qd1);
% 
% yd = [qd1; qd2; theta_dd; Td; Ld];

yd = data.h_Fyd(t, y, u);

end

