function plot_pendulum(t, y, par)
q = y(:, 1:2);
h = figure(1);
for n = 1:size(q, 1)
x1 = [0, -par.l1*sin(q(n, 1))];
y1 = [0, par.l1*cos(q(n, 1))];
x2 = x1(2) - [0, par.l2*sin(q(n, 1) + q(n, 2))];
y2 = y1(2) + [0, par.l2*cos(q(n, 1) + q(n, 2))];

plot(x1, y1, x2, y2, 'LineWidth', 2)
axis([-0.4,0.4,-0.4,0.4])
drawnow
if n < size(q, 1)
    pause(t(n+1)-t(n))
end
end

end

