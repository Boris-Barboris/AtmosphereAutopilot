%% prepare
run('import_telemetry.m');

%% plot
nn_inputs = [aoa; control; airspd; p];
nn_outputs = smoothed_acc;
plot(time, acc, 'r')
hold on
plot(time, nn_outputs, 'r:')
plot(time, myNeuralNetworkFunction(nn_inputs), 'b')
hold off
xlabel('time')
legend('acc','smoothed acc','ann acc');

