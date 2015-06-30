%% prepare
run('import_telemetry.m');

%% plot
nn_inputs = [aoa; control];
nn_outputs = acc;
plot(time, acc, 'r')
hold on
plot(time, nn_outputs, 'r:')
plot(time, myNeuralNetworkFunction(nn_inputs), 'b')
hold off
xlabel('time')
legend('acc','smoothed acc','ann acc');
%% plot control sensitivity
figure('Name','control sensitivity')
plot(linspace(-1,1,99), myNeuralNetworkFunction([zeros(1,99);linspace(-1,1,99)]));
xlabel('control');
ylabel('acc');
%% plot aoa sensitivity
figure('Name','aoa sensitivity')
plot(linspace(-0.25,0.25,99), myNeuralNetworkFunction([linspace(-0.25,0.25,99); zeros(1,99)]));
xlabel('aoa');
ylabel('acc');