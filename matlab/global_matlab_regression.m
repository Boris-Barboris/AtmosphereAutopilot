%% prepare
run('import_telemetry.m');

%% regression

% initial values
k_aoa = 0.0;
b_aoa = 0.0;
k_input = 100.0;
moi = 58.0;

%% problem creation
lower_bound = [-Inf,-Inf,0.0];
upper_bound = [Inf,Inf,Inf];
start_point = [k_aoa,b_aoa,k_input];
problem = createOptimProblem('fmincon','x0',start_point,...
    'objective',@(x)error_function(x(1),x(2),x(3),moi,acc,aoa,control,1.0),...
    'lb',lower_bound,'ub',upper_bound);

%% solver creation
gs = GlobalSearch('MaxTime', 60);

%% run solver
[xmin,fming,flagg,outptg,manyminsg] = run(gs,problem)

%% get regressed signal
regressed_acc = angular_model(moi, xmin(1), xmin(2), aoa,...
        xmin(3), control);

%% plot
scrsz = get(0,'ScreenSize');
figure('Name','Telemetry Plotter',...
    'Position',[100 50 scrsz(3)*0.8 scrsz(4)*0.8])
plot(time,acc,'r','Marker','.','MarkerSize',5)
hold on
plot(time,control,'k','Marker','.','MarkerSize',5)
plot(time,aoa,'b','Marker','.','MarkerSize',5)
plot(time,regressed_acc,'r:')
plot(time,v,'g')
%plot(time,acc_smooth,'c')
hold off
xlabel('time')
legend('acc-smooth','truecontrol','aoa','regressed','v');
h = gca;
set(h, 'Position', [0.02 0.06 0.96 0.92]);