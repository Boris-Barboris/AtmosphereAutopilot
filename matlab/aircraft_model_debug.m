model = aircraft_model();
sim_length = 100;
time = zeros(1, sim_length);
positions = zeros(3, sim_length);
forwards = zeros(3, sim_length);
rights = zeros(3, sim_length);
for frame = 1:sim_length
    time(frame) = 0.025 * (frame - 1);
    model.preupdate();
    model.simulation_step(0.025, zeros(1, 3));
    positions(:, frame) = model.position.';
    forwards(:, frame) = model.forward_vector.';
    rights(:, frame) = model.right_vector.';
end
%% Drawing
plot3(positions(1, :), positions(2, :), positions(3, :), 'b');
hold on;
quiver3(positions(1, :), positions(2, :), positions(3, :), forwards(1, :), forwards(2, :), forwards(3, :), 0.5, '.r');
hold off;
axis([-100 100 -inf inf -inf inf])
grid on;
xlabel('x');
ylabel('y');
zlabel('height');