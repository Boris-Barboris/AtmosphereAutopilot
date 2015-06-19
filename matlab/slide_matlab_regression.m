%% start

% initial values
k_aoa = -1000.0;
b_aoa = 100.0;
k_input = 300.0;
k_v = -100.0;
moi = 58.0;

slider_length = int16(10);  % length of signal sampling frame

%% problem creation
lower_bound = [-Inf,-Inf,0.0,-Inf];
upper_bound = [Inf,Inf,Inf,0.0];
start_point = [k_aoa,b_aoa,k_input,k_v];

frame_index = int16(1);
acc_frame = acc(frame_index:slider_length);
aoa_frame = aoa(frame_index:slider_length);
true_control_frame = true_control(frame_index:slider_length);
v_frame = v(frame_index:slider_length);
regressed_acc = zeros(1,length(acc));

problem = createOptimProblem('fmincon','x0',start_point,...
    'objective',@(x)error_function(x,moi,acc_frame,aoa_frame,true_control_frame,v_frame),...
    'lb',lower_bound,'ub',upper_bound);

%% solver creation
gs = GlobalSearch('MaxTime', 60);

%% run solver
% first run
xmin = run(gs,problem);
regressed_acc(1:slider_length) = angular_model(moi, xmin(1), xmin(2), aoa_frame,...
        xmin(3), true_control_frame, xmin(4), v_frame);
for frame_index = (frame_index+1): (length(acc)-slider_length+1)
    acc_frame = acc(frame_index:frame_index+slider_length-1);
    aoa_frame = aoa(frame_index:frame_index+slider_length-1);
    true_control_frame = true_control(frame_index:frame_index+slider_length-1);
    v_frame = v(frame_index:frame_index+slider_length-1);
    problem.x0 = xmin;
    problem.objective = @(x)error_function(x,moi,acc_frame,aoa_frame,true_control_frame,v_frame);
    xmin = run(gs,problem);
    regressed_acc(frame_index+slider_length-1) =...
        angular_model(moi, xmin(1), xmin(2), aoa_frame(slider_length),...
        xmin(3), true_control_frame(slider_length), xmin(4),...
        v_frame(slider_length));
end

%% plot
scrsz = get(0,'ScreenSize');
figure('Name','Telemetry Plotter',...
    'Position',[100 50 scrsz(3)*0.8 scrsz(4)*0.8])
plot(time,acc,'r','Marker','.','MarkerSize',5)
hold on
plot(time,true_control,'k','Marker','.','MarkerSize',5)
plot(time,aoa,'b','Marker','.','MarkerSize',5)
plot(time,regressed_acc,'r:','MarkerSize',5)
plot(time,v,'g')
hold off
xlabel('time')
legend('acc','truecontrol','aoa','regressed','v');
h = gca;
set(h, 'Position', [0.02 0.06 0.96 0.92]);