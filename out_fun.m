function stop = out_fun(X, optimValues, state, hFig, par, data, h_sim_fun)
     stop = false;
     nDisControls = data.nDisControls;
     nDisStates = data.nDisStates;
     statesSize = data.statesSize;
     
     switch state
         case 'init'
            clf(hFig);
         case 'iter'
            set(0,'CurrentFigure',hFig)
            
            % Get distcrete controls and states
            u_dis = X(1:nDisControls);
            T = X(nDisControls + 1);
            intervalTimes = linspace(0, T, nDisStates);
            y_dis = reshape(X(nDisControls+2:end), statesSize);
            [t, y, u] = h_sim_fun(X);
            
            % Controls
            subplot(2,2,1)
            plot(t, u'); % Interpolated
            hold on
            plot(linspace(0,T,nDisControls)', u_dis','ko','MarkerSize',4); % States
            xlabel('Time [s]')
            title('Interpolated controls')
            axis tight
            hold off
            
            % Angles
            subplot(2,2,2)
            plot(t, y(:,1:2))
            hold on
            plot(intervalTimes', y_dis(1:2,:)','ko','MarkerSize',4);
            hold off
            xlabel('Time [s]')
            ylabel('Angle [rad]')
            title('Joint angles')
            
            % Angles speed
            subplot(2,2,3)
            plot(t, y(:,3:4))
            xlabel('Time [s]')
            ylabel('Angular velocity [rad/s]')
            title('Angular velocity')
            
            subplot(2,2,4)
            q = y(end, 1:2);
            x1 = [0, -par.l1*sin(q(1))];
            y1 = [0, par.l1*cos(q(1))];
            x2 = x1(2) - [0, par.l2*sin(q(1) + q(2))];
            y2 = y1(2) + [0, par.l2*cos(q(1) + q(2))];
            plot(x1, y1, x2, y2, 'LineWidth', 2)
            axis([-0.25,0.25,-0.25,0.25])
            xlabel('x [m]')
            ylabel('y [m]')
            
            drawnow
         case 'done'
             hold off
         otherwise
     end
     
     
end