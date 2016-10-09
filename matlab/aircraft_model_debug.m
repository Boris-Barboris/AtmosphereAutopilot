clear;
model = aircraft_model();
model.aero_model = false;
model.force_spd_maintain = true;
sim_length = 100;
dt = 0.05;
time = zeros(1, sim_length);
positions = zeros(3, sim_length);
forwards = zeros(3, sim_length);
rights = zeros(3, sim_length);
aoas = zeros(3, sim_length);
speed = zeros(1, sim_length);
csurf = zeros(3, sim_length);
for frame = 1:sim_length
    time(frame) = dt * (frame - 1);
    model.preupdate(dt);
    model.simulation_step(dt, [0, 0, 0]);
    positions(:, frame) = model.position.';
    forwards(:, frame) = model.forward_vector.';
    rights(:, frame) = model.right_vector.';
    aoas(:, frame) = model.aoa.';
    speed(frame) = model.velocity_magn;
    csurf(:, frame) = model.csurf_state.';
end
%% Drawing
plot3(positions(1, :), positions(2, :), positions(3, :), 'b');
hold on;
%quiver3(positions(1, :), positions(2, :), positions(3, :), forwards(1, :), forwards(2, :), forwards(3, :), 0.5, '.r');
%quiver3(positions(1, :), positions(2, :), positions(3, :), rights(1, :), rights(2, :), rights(3, :), 0.5, '.g');
hold off;
axis([-500 500 -inf inf 400 600])
grid on;
xlabel('x');
ylabel('y');
zlabel('height');
%% Graph
h = figure(1337);
plot(time, aoas(1, :));
xlabel('time');
ylabel('AoA');
%% CUDA
ksp_plots_path ='D:\Coding\В разработке\KSP_mods\AtmosphereAutopilot\AAGpu\';
cuda_aoa = csvread([ksp_plots_path, 'simul.csv']);
hold on
plot(time, cuda_aoa(1, :), 'r');
hold off
%% Graph
h = figure(1338);
plot(time, speed(1, :));
xlabel('time');
ylabel('airspeed');
%%
h = figure(1339);
plot(time, csurf(1, :), 'r');
hold on;
plot(time, csurf(2, :), 'g');
plot(time, csurf(3, :), 'b');
hold off;
xlabel('time');
ylabel('csurf');
legend('pitch','roll','yaw');