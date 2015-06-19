%% data preparation
run('import_telemetry.m');

%% prepare sliding frames
frame_index = int16(1);     % current frame tail
buffer_size = int16(100);   % length of state space memory
short_frame_size = int16(20);     % size of frame for k_acc and k_input
short_fixed_size = int16(20);     % size of deterministic frame section
tick_limit = int16(300);
descend_k = 0;
miss_k = 0.05;
tolerance = 1e-2;
recent_factor = 1;

rng('default');

% prepare performace analysis vectors
ticks = int16(zeros(1,series_length));      % count error function evaluations
error_values = zeros(1,series_length);      % error function final value on this slide
param_values = zeros(4,series_length);      % regressed parameter vectors

%% prepare initial conditions
param_values(:,1) = [0,0,50,0].';
probe_deltas = [0.1,1e-4,0.1,0.1];
grad_mask = [1,1e-6,1,1];
constraints = [-Inf Inf; -Inf Inf; 0 Inf; -Inf 0];
moi = 58;

%% run regressor

% cycle on new input
for frame_index = 1 : series_length            
    % update error function
    e_func = @(x,frame)error_function(...
        x(1),x(2),x(3),x(4),...
        moi,frame(1,:),frame(2,:),frame(3,:),frame(4,:),recent_factor);
    % update frame
    frame = get_frame(state_space,buffer_size,frame_index,short_frame_size,short_fixed_size);
    % do regression
    [param_values(:,frame_index), error_values(frame_index), ticks(frame_index)] =...
        frame_grad_descend(e_func, param_values(:,frame_index).', constraints, probe_deltas,...
        grad_mask, frame, tick_limit, descend_k, miss_k, tolerance);
    
    % propagate solution forward
    if (frame_index < series_length)
        param_values(:,frame_index + 1) = param_values(:,frame_index);
    end
end

%% compute model acceleration
model_acc = angular_model(moi,param_values(1,:),param_values(2,:),...
    aoa,param_values(3,:),control,param_values(4,:),v);
predict_2 = [0.0, 0.0, angular_model(moi,param_values(1,1:series_length-2),...
    param_values(2,1:series_length-2),...
    state_space(2,3:end),param_values(3,1:series_length-2),state_space(3,3:end),...
    param_values(4,1:series_length-2),state_space(4,3:end))];


%% output main plot
scrsz = get(0,'ScreenSize');
figure('Name','Telemetry Plotter',...
    'Position',[100 50 scrsz(3)*0.8 scrsz(4)*0.8])
plot(time,smoothed_acc,'r','Marker','.','MarkerSize',5)
hold on
%plot(time,smoothed_acc,'r-')
plot(time,control,'k','Marker','.','MarkerSize',5)
plot(time,aoa,'b','Marker','.','MarkerSize',5)
plot(time,model_acc,'k:')
plot(time,predict_2,'b:')
%plot(time,model_predict_acc,'b:')
hold off
xlabel('time')
legend('smooth_acc','control','aoa','model acc','model prediction');
h = gca;
set(h, 'Position', [0.02 0.06 0.96 0.92]);