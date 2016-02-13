clear;
model = aircraft_model();
%model.force_spd_maintain = true;
sim_length = 500;
time = zeros(1, sim_length);
positions = zeros(3, sim_length);
forwards = zeros(3, sim_length);
rights = zeros(3, sim_length);
aoas = zeros(3, sim_length);
speed = zeros(1, sim_length);
for frame = 1:sim_length
    time(frame) = 0.05 * (frame - 1);
    model.preupdate();
    model.simulation_step(0.025, [0.06, 0, 0]);
    positions(:, frame) = model.position.';
    forwards(:, frame) = model.forward_vector.';
    rights(:, frame) = model.right_vector.';
    aoas(:, frame) = model.aoa.';
    speed(frame) = model.velocity_magn;
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
%% Graph
h = figure(1338);
plot(time, speed(1, :));
xlabel('time');
ylabel('airspeed');