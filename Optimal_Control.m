clear
% Optimal Control of a dubble pendulum
% y = [theta_1; theta_2; theta_1d; theta_2d; T];

% Load parameters
getParameters;

nDisControls = 15;
nDisStates = 1;
data.nDisControls = nDisControls;
data.nDisStates = nDisStates;
data.interpMethod = 'linear';

% Initial guess
u0 = 0.5*ones(nDisControls, 1);
y0 = [pi; 0; 0; 0; 0];
X0 = [u0; y0];

% Linear Constraints (Aeq*X  = Beq)
Aeq = zeros(2, length(X0));
Aeq(1, nDisControls+1) = 1;
Aeq(2, nDisControls+2) = 1;
Beq = [pi; 0];

% Bounds
LB = [-1 * ones(1, nDisControls), -10 * ones(1, 5*nDisStates)];
UB = [ 1 * ones(1, nDisControls),  10 * ones(1, 5*nDisStates)];

%% Run optimization
options = optimoptions('fmincon',...
    'Display','iter-detailed',...
    'Algorithm','sqp');
X = fmincon(@(X) obj_fun(X, par, data), X0, [], [], Aeq, Beq, LB, UB, @(X) con_fun(X, par, data), options);

%% Plot
% Get distcrete controls and states
u_dis_sol = X(1:data.nDisControls);
y0_sol = X(data.nDisControls+1:end);

% Simulate system
[t, y] = ode45(@(t, y) ODEFUN(t, y, u_dis_sol, par, data), [0 5], y0_sol);

plot_pendulum(t, y, par)

figure(2)
plot(t, y(:,1:2), linspace(0,5,nDisControls), u_dis_sol)


