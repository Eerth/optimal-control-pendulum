function [h_obj_fun, h_con_fun] = obj_con_fun(data, par)

h_obj_fun = @obj_fun;
h_con_fun = @con_fun;

X_local = [];
y = []; t = [];
u = [];
nDisStates = data.nDisStates;
intervalTimes = data.intervalTimes;
nDisControls = data.nDisControls;
statesSize = data.statesSize;
y_dis = [];
y_interval_end = [];

function [L] = obj_fun(X, par, data)
    
    if ~isequal(X, X_local)
        % Get distcrete controls and states
        u_dis = X(1:nDisControls);
        y_dis = reshape(X(nDisControls+1:end), statesSize);
        
        % Simulate system
        y = []; t = [];
        for n = 1:nDisStates-1
            [t_interval, y_interval] = ode45(@(t, y) ODEFUN(t, y, u_dis, par, data), [intervalTimes(n) intervalTimes(n+1)], y_dis(:,n));
            y_interval_end(n,:) = y_interval(end,:);
            y = [y; y_interval];
            t = [t; t_interval];
        end
        % Interpolate u
        u = interp1(linspace(0,5,data.nDisControls), u_dis, t, data.interpMethod);
        
        X_local = X;
    end
    
    % Objective
    L = (u'*u);
    
end

function [c, ceq] = con_fun(X, par, data)

    if ~isequal(X, X_local)
        % Get distcrete controls and states
        u_dis = X(1:nDisControls);
        y_dis = reshape(X(nDisControls+1:end), statesSize);

        % Simulate system
        [t, y] = ode45(@(t, y) ODEFUN(t, y, u_dis, par, data), [0 5], y_dis(:,1));

        % Interpolate u
        u = interp1(linspace(0,5,nDisControls), u_dis, t, data.interpMethod);
        
        X_local = X;
    end
    
    %Continuity constraints
    ceq = reshape(y_dis(:,2:end) - y_interval_end', [], 1);
    c = [];
    
end

end

