%% import time series with removed delays
ksp_plots_path ='C:\Games\Kerbal Space Program 0.90\Resources\';

acc = csvread([ksp_plots_path, 'acc.csv']);
control = [0.0,0.0,csvread([ksp_plots_path, 'control.csv'])];
aoa = [0.0,csvread([ksp_plots_path, 'aoa.csv'])];
v = [0.0,csvread([ksp_plots_path, 'v.csv'])];

smoothed_acc = sgolayfilt(acc, 3, 11);

%% cut edges from telemetry and prepare time axis
max_length = max([length(acc), length(control), length(aoa), length(v)]);
time = 0:0.02:(0.02 * (max_length - 6));

acc = [acc,zeros(1, max_length - length(acc))];
acc = acc(3:max_length - 3);
aoa = [aoa, zeros(1, max_length - length(aoa))];
aoa = aoa(3:max_length - 3);
control = [control, zeros(1,max_length - length(control))];
control = control(3:max_length - 3);
v = [v,zeros(1, max_length - length(v))];
v = v(3:max_length - 3);

smoothed_acc = [smoothed_acc,zeros(1, max_length - length(smoothed_acc))];
smoothed_acc = smoothed_acc(3:max_length - 3);

state_space = [acc; aoa; control; v];

%% ticks limit testing
ticks_limit = 40:20:300;
err = zeros(1,length(ticks_limit));
parfor i = 1:length(ticks_limit)
    err(i) = run_descender(ticks_limit(i), 100, 2, 2, 0, 0.1, state_space);
end
plot(ticks_limit,err);

%% frame size testing
frame_size_axis = 1:1:10;
err = zeros(1,length(frame_size_axis));
parfor i = 1:length(frame_size_axis)
    err(i) = run_descender(int16(300), 100, frame_size_axis(i), frame_size_axis(i),...
        0.0, 0.1, 1, 1e-2, state_space, smoothed_acc);
end
plot(frame_size_axis,err);
%% recent_factor_testing
recent_factor = 1:10:100;
err = zeros(1,length(recent_factor));
parfor i = 1:length(recent_factor)
    err(i) = run_descender(int16(300), 100, 2, 2,...
        0.0, 0.1, recent_factor(i), 1e-2, state_space, smoothed_acc);
end
plot(recent_factor,err);

%% descend_k testing
descend_k_axis = linspace(-1000, 0, 50);
err = zeros(1,length(descend_k_axis));
parfor i = 1:length(descend_k_axis)
    err(i) = run_descender(int16(150), 100, 2, 2, descend_k_axis(i), 0.09, state_space);
end
plot(descend_k_axis,err);

%% missshot_k testing
misshot_k_axis = linspace(0.01, 0.7, 100);
err = zeros(1,length(misshot_k_axis));
parfor i = 1:length(misshot_k_axis)
    err(i) = run_descender(int16(150), 100, 2, 2, 0, misshot_k_axis(i), state_space);
end
plot(misshot_k_axis,err);