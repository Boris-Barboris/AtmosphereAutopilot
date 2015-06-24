%% plot generalization space
figure('Name','Generalization space')
scatter3(gen_inputs(1,1:j), gen_inputs(2,1:j), gen_outputs(1:j));
xlabel('AoA');
ylabel('control');
zlabel('acc');
%% plot control sensitivity
figure('Name','control sensitivity')
plot(linspace(-1,1,99), anneval_large([zeros(1,99); linspace(-1,1,99)],...
    weights, biases, input_count, hidden_count));
xlabel('control');
ylabel('acc');
%% plot aoa sensitivity
figure('Name','aoa sensitivity')
plot(linspace(-0.2,0.2,99), anneval_large([linspace(-0.2,0.2,99); zeros(1,99)],...
    weights, biases, input_count, hidden_count));
xlabel('aoa');
ylabel('acc');
%% cumulative acceleration graph
frame = 650;
l = 32;
x = 1:l;
y = zeros(1,l);
y_model = y;
y_smoothed = y;
for i=1:l
    y(i) = sum(norm_acc(:,(frame-i+1):frame));
    y_model(i) = sum(ann_global_outputs(:,(frame-i+1):frame));
    y_smoothed(i) = sum(norm_smoothed_acc(:,(frame-i+1):frame));
end
figure('Name','Cumulative acc');
plot(x,y,'-b*');
hold on
plot(x,y_model,'-r*');
plot(x,y_smoothed,'-k*');
hold off
legend('acc','model acc','smoothed acc');
%% find optimal generalization koeffitients
[decay, factor] = meshgrid(0.5:0.5:5, 0.1:0.1:1);
performance = arrayfun(@ann_flight_f, decay, factor);
mesh(decay,factor,performance)
xlabel('decay')
ylabel('importance')