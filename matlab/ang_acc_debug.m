clear;
model = aircraft_model();
model.aero_model = true;        % far model
%model.angular_vel(3) = 0.1;
model.force_spd_maintain = false;
pitch_acc_c = ang_acc_pitch_yaw(0, model);
roll_acc_c = ang_acc_roll(model);
dt = 0.05;
sim_length = 100;
time = zeros(1, sim_length);
positions = zeros(3, sim_length);
forwards = zeros(3, sim_length);
rights = zeros(3, sim_length);
aoas = zeros(3, sim_length);
speed = zeros(1, sim_length);
ang_acc = zeros(3, sim_length);
ang_vel = zeros(3, sim_length);
cntrl = zeros(3, sim_length);
csurf = zeros(3, sim_length);
for frame = 1:sim_length
    time(frame) = dt * (frame - 1);
    model.preupdate(dt);
    p_output = pitch_acc_c.eval(0.0, dt);
    y_output = 0.0;
    if (time(frame) > 2 && time(frame) < 2.5)
        des_roll_acc = 0.0;
    else
        des_roll_acc = 0.0;
    end
    r_output = roll_acc_c.eval(des_roll_acc, y_output, dt);
    
    cntrl(:, frame) = [p_output, r_output, 0];
    model.simulation_step(dt, cntrl(:, frame));
    
    positions(:, frame) = model.position.';
    forwards(:, frame) = model.forward_vector.';
    rights(:, frame) = model.right_vector.';
    aoas(:, frame) = model.aoa.';
    speed(frame) = model.velocity_magn;
    csurf(:, frame) = model.csurf_state;
    ang_acc(:, frame) = model.angular_acc;
    ang_vel(:, frame) = model.angular_vel;
end
%% Drawing
h = figure(1336);
plot3(positions(1, :), positions(2, :), positions(3, :), 'b');
hold on;
%quiver3(positions(1, :), positions(2, :), positions(3, :), forwards(1, :), forwards(2, :), forwards(3, :), 0.5, '.r');
%quiver3(positions(1, :), positions(2, :), positions(3, :), rights(1, :), rights(2, :), rights(3, :), 0.5, '.g');
hold off;
axis([-inf inf -inf inf -inf inf])
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
plot(time, ang_acc(1,:), 'b');
hold on
plot(time, cntrl(1,:), 'r');
plot(time, csurf(1,:), 'k:');
hold off
xlabel('time');
legend('acc', 'pitch c', 'pitch csurf');
%% Graph
h = figure(1339);
plot(time, ang_acc(2,:), 'b');
hold on
plot(time, ang_vel(2,:), 'g');
plot(time, aoas(3,:), 'm.');
plot(time, cntrl(2,:), 'r');
plot(time, csurf(2,:), 'k:');
hold off
xlabel('time');
legend('acc', 'roll ang vel', 'aoa yaw', 'roll c', 'roll csurf');