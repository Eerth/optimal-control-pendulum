clear
% Optimal Control of a dubble pendulum
% y = [theta_1; theta_2; theta_1d; theta_2d; T];

% Load model parameters
getParameters;

% Hyper parameters
nDisControls = 15;
nDisStates = 5;
statesSize = [5, nDisStates];
T = 5; % End time
intervalTimes = linspace(0, T, nDisStates);
data.interpMethod = 'previous'; % linear, spline

% Save in data
data.nDisControls = nDisControls;
data.nDisStates = nDisStates;
data.statesSize = statesSize;
data.T = T;
data.intervalTimes = intervalTimes;

% Initial guess
u_init = 0.5*ones(nDisControls, 1);

q0 = [pi; 0];
qf = [0; 0];
q_init = [linspace(q0(1), qf(1), nDisStates); linspace(q0(2), qf(2), nDisStates)];
qd_init = gradient(q_init);
tau_init = zeros(1, nDisStates);
y_init = [q_init; qd_init; tau_init];

% Combine optimization variables into one vector
X_init = [u_init; y_init(:)];

% Linear constraints (Aeq*X  = Beq)
Aeq = zeros(2, length(X_init));
Aeq(1, nDisControls+1) = 1;
Aeq(2, nDisControls+2) = 1;
Aeq(3, nDisControls+3) = 1;
Aeq(4, nDisControls+4) = 1;
Aeq(5, length(X_init)-4) = 1;
Aeq(6, length(X_init)-3) = 1;
Aeq(7, length(X_init)-2) = 1;
Aeq(8, length(X_init)-1) = 1;
Beq = [pi; 0; 0; 0; 0; 0; 0; 0];

% Bounds
LB = [-1 * ones(1, nDisControls), -100 * ones(1, 5*nDisStates)];
UB = [ 1 * ones(1, nDisControls),  100 * ones(1, 5*nDisStates)];

%% Run optimization
[h_obj_fun, h_con_fun] = obj_con_fun(data, par);
options = optimoptions('fmincon',...
    'Display','iter-detailed',...
    'MaxIterations',30,...
    'Algorithm','sqp');
X = fmincon(@(X) h_obj_fun(X, par, data), X_init, [], [], Aeq, Beq, LB, UB, @(X) h_con_fun(X, par, data), options);

%% Plot
% Get distcrete controls and states
u_dis = X(1:data.nDisControls);
y_dis = reshape(X(nDisControls+1:end),statesSize);

% Simulate system
[t, y] = ode45(@(t, y) ODEFUN(t, y, u_dis, par, data), [0 5], y_dis(:,1));
u = interp1(linspace(0,5,data.nDisControls), u_dis, t, data.interpMethod);

plot_pendulum(t, y, par)

figure(2)
plot(t, y(:,1:5), t, u)
legend('q1','q2','qd1','qd2','tau');

