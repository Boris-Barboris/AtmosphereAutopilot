%% data
delta_time = 0.025;
run('import_telemetry');
%% plot

scrsz = get(0,'ScreenSize');
figure('Name','Telemetry Plotter',...
    'Position',[100 50 scrsz(3)*0.9 scrsz(4)*0.8])
hold on
plot(time, 1.0 .* acc,'r','Marker','.','MarkerSize',5)
plot(time, 1.0 .* predict,'r-.')
plot(time, 1.0 * control,'k','Marker','.','MarkerSize',5)
plot(time, 1.0 * output,'k:','Marker','.','MarkerSize',5)
plot(time, 1.0 .* aoa,'b','Marker','.','MarkerSize',5)
plot(time, 5.0 .* v,'g')
hold off
xlabel('time')
legend('acc','predict','csurf','control','aoa','v');
h = gca;
set(h, 'Position', [0.025 0.06 0.96 0.92]);