function [h_obj_fun, h_con_fun, h_sim_fun] = obj_con_fun(data, par)

h_obj_fun = @obj_fun;
h_con_fun = @con_fun;
h_sim_fun = @sim_fun;

ode_options = odeset('Jacobian', data.h_Fjac);

X_local = [];
y = []; t = []; u = []; T = [];
nDisStates = data.nDisStates;
nDisControls = data.nDisControls;
statesSize = data.statesSize;
y_dis = []; u_dis = [];
y_interval_end = [];

function [L, L_grad] = obj_fun(X)
    
    if ~isequal(X, X_local)
        [t, y, u, y_interval_end] = sim_fun(X);
        
        X_local = X;
    end
    
    % Objective
    L = y(end, end);
    
    % Optional gradient (set 'SpecifyObjectiveGradient' to true)
    if nargout > 1
        delta = sqrt(eps);
        for i = 1:length(X)
            X_dist = X;
            X_dist(i) = X_dist(i) + delta;
            [~, y_dist] = sim_fun(X_dist);

            % Cost gradient
            L_grad(i) = (y_dist(end, end) - L) / delta;
        end
    end
    
end

function [c, ceq] = con_fun(X)

    if ~isequal(X, X_local)
        [t, y, u, y_interval_end] = sim_fun(X);
        
        X_local = X;
    end
    
    %Continuity constraints
    ceq = reshape(y_dis(:,2:end) - y_interval_end', [], 1);
    
    c = [];
    
end

function [t, y, u, y_interval_end] = sim_fun(X)

    % Get distcrete controls and states
    u_dis = X(1:nDisControls);
    T = X(nDisControls + 1);
    intervalTimes = linspace(0, T, nDisStates);
    y_dis = reshape(X(nDisControls+2:end), statesSize);
    
    % Simulate system
    y = []; t = [];
    for n = 1:nDisStates-1
        [t_interval, y_interval] = ode45(@(t, y) ODEFUN(t, y, u_dis, T, par, data), [intervalTimes(n) intervalTimes(n+1)], [y_dis(:,n); 0], ode_options);
        y_interval_end(n,:) = y_interval(end,1:5);
        y = [y; y_interval];
        t = [t; t_interval];
    end
    
    % Interpolate u
    u = interp1(linspace(0,T,data.nDisControls), u_dis, t, data.interpMethod);

end

end

