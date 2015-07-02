%% plot generalization space
figure('Name','Generalization space')
scatter3(gen_inputs(1,1:k), gen_inputs(2,1:k), gen_outputs(1:k));
xlabel('AoA');
ylabel('control');
zlabel('acc');
%% plot control sensitivity
figure('Name','control sensitivity')
control_d = linspace(-1,1,99);
aoa_d = zeros(1,99);
linmodel = zeros(1, length(control_d));
for frame = 1:99
    linmodel(frame) = fittedmodel(aoa_d(frame), control_d(frame));
end
plot(control_d, anneval_large([aoa_d; control_d],...
    weights, biases, input_count, hidden_count))
hold on
plot(control_d, linmodel);
hold off
xlabel('control');
ylabel('acc');
%% plot aoa sensitivity
figure('Name','aoa sensitivity')
aoa_d = linspace(-0.5,0.5,99);
control_d = zeros(1,99);
linmodel = zeros(1, length(control_d));
for frame = 1:99
    linmodel(frame) = fittedmodel(aoa_d(frame), control_d(frame));
end
plot(aoa_d, anneval_large([aoa_d; control_d],...
    weights, biases, input_count, hidden_count))
hold on
plot(aoa_d, linmodel);
hold off;
xlabel('aoa');
ylabel('acc');
%% plot ann surface
aoa_d = linspace(-4,4,99);
control_d = linspace(-1,1,99);
[Aoa, Cntrl] = meshgrid(aoa_d, control_d);
Acc = zeros(99, 99);
for i = 1:99
    for k = 1:99
        Acc(i,k) = anneval_large([Aoa(i,k), Cntrl(i,k)].',...
    weights, biases, input_count, hidden_count);
    end
end
surf(Aoa,Cntrl,Acc);
hold on
scatter3(global_inputs(1,:), global_inputs(2,:), global_outputs(1,:));
hold off
xlabel('Aoa');
ylabel('Cntrl');
%% cumulative acceleration graph
frame = 1000;
l = 32;
x = 1:l;
y = zeros(1,l);
y_model = y;
y_smoothed = y;
for i=1:l
    y(i) = sum(norm_acc(:,(frame-i+1):frame));
    y_model(i) = sum(ann_global_outputs(:,(frame-i+1):frame));
end
figure('Name','Cumulative acc');
plot(x,y,'-b*');
hold on
plot(x,y_model,'-r*');
hold off
legend('acc','model acc');
%% find optimal generalization koeffitients
[decay, factor] = meshgrid(0.5:0.5:5, 0.1:0.1:1);
performance = arrayfun(@ann_flight_f, decay, factor);
mesh(decay,factor,performance)
xlabel('decay')
ylabel('importance')