plot(time(1:iter), aoa(1:iter), 'r')
hold on
plot(time(1:iter), ang_vel(1:iter), 'b')
plot(time(1:iter), csurf(1:iter), 'k:')
plot(time(1:iter), input(1:iter), 'k')
hold off
legend('aoa', 'v', 'csurf', 'input');