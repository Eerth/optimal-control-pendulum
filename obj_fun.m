function [L] = obj_fun(X, par, data)

% Get distcrete controls and states
u_dis = X(1:data.nDisControls);
y_dis = reshape(X(nDisControls+1:end),statesSize);

% Simulate system
[t, y] = ode45(@(t, y) ODEFUN(t, y, u_dis, par, data), [0 5], y0);

% Interpolate u
u = interp1(linspace(0,5,data.nDisControls), u_dis, t, data.interpMethod);

% Objective
L = (u'*u);

end

