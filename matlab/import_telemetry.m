%% import time series with removed delays
ksp_plots_path ='D:\Games\Kerbal Space Program 1.1\Resources\';

acc = csvread([ksp_plots_path, 'acc.csv']);
control = csvread([ksp_plots_path, 'control.csv']);
output = [0.0,csvread([ksp_plots_path, 'output.csv'])];
aoa = csvread([ksp_plots_path, 'aoa.csv']);
v = csvread([ksp_plots_path, 'v.csv']);
predict = [0.0,csvread([ksp_plots_path, 'predict.csv'])];
airspd = [0.0,csvread([ksp_plots_path, 'airspd.csv'])];
p = [0.0,csvread([ksp_plots_path, 'density.csv'])];

smoothed_acc = sgolayfilt(acc, 2, 11);

%% cut edges from telemetry and prepare time axis
max_length = max([length(acc), length(control), length(output), length(aoa),...
    length(v), length(airspd), length(p), length(predict)]);
if ~exist('delta_time', 'var')
    delta_time = 0.025;
end
time = 0:delta_time:(delta_time * (max_length - 6));

acc = [acc,zeros(1, max_length - length(acc))];
acc = acc(3:max_length - 3);
aoa = [aoa, zeros(1, max_length - length(aoa))];
aoa = aoa(3:max_length - 3);
control = [control, zeros(1,max_length - length(control))];
control = control(3:max_length - 3);
output = [output, zeros(1,max_length - length(output))];
output = output(3:max_length - 3);
v = [v,zeros(1, max_length - length(v))];
v = v(3:max_length - 3);
predict = [predict,zeros(1, max_length - length(predict))];
predict = predict(3:max_length - 3);
airspd = [airspd,zeros(1, max_length - length(airspd))];
airspd = airspd(3:max_length - 3);
p = [p,zeros(1, max_length - length(p))];
p = p(3:max_length - 3);

smoothed_acc = [smoothed_acc,zeros(1, max_length - length(smoothed_acc))];
smoothed_acc = smoothed_acc(3:max_length - 3);

series_length = length(acc);