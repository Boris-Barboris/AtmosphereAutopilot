mpc_time = linspace(0.0, iter * mpc_dt, iter);
plot(mpc_time, aoa(1:iter), 'r')
hold on
plot(mpc_time, ang_vel(1:iter), 'b')
plot(mpc_time, csurf(1:iter), 'k:')
plot(mpc_time, input(1:iter), 'k')
hold off
legend('aoa', 'v', 'csurf', 'input');