%% import time series with removed delays
ksp_plots_path ='D:\Games\Kerbal Space Program 0.90\Resources\';

acc = csvread([ksp_plots_path, 'acc.csv']);
control = [0.0,0.0,csvread([ksp_plots_path, 'control.csv'])];
aoa = [0.0,csvread([ksp_plots_path, 'aoa.csv'])];
v = [0.0,csvread([ksp_plots_path, 'v.csv'])];
predict = [0.0,csvread([ksp_plots_path, 'predict.csv'])];
airspd = [0.0,csvread([ksp_plots_path, 'airspd.csv'])];
p = [0.0,csvread([ksp_plots_path, 'density.csv'])];

smoothed_acc = sgolayfilt(acc, 2, 11);

%% cut edges from telemetry and prepare time axis
max_length = max([length(acc), length(control), length(aoa),...
    length(v), length(airspd), length(p), length(predict)]);
time = 0:0.02:(0.02 * (max_length - 6));

acc = [acc,zeros(1, max_length - length(acc))];
acc = acc(3:max_length - 3);
aoa = [aoa, zeros(1, max_length - length(aoa))];
aoa = aoa(3:max_length - 3);
control = [control, zeros(1,max_length - length(control))];
control = control(3:max_length - 3);
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