clear;
model = aircraft_model();
model.aero_model = true;         % stock model
model.force_spd_maintain = true;
pitch_acc_c = ang_acc_pitch_yaw(0, model);
pitch_vel_c = ang_vel_pitch_yaw(0, pitch_acc_c);
dt = 0.05;
sim_length = 50;
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
    if (time(frame) > 0.0 && time(frame) < 1.0)
        des_pitch_v = -0.4;
    else
        des_pitch_v = 0.0;
    end
    p_output = pitch_vel_c.eval(des_pitch_v, 0.0, dt);
    cntrl(:, frame) = [p_output, 0, 0];
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
% Plot pitch parameters
h = figure(1338);
hold on
H0 = plot(time, ang_acc(1,:), 'b');
[AX, H1, H2] = plotyy(time, ang_vel(1,:), time, aoas(1, :));
set(H1, 'Color', 'g');
set(H2, 'Color', 'm');
H3 = plot(time, cntrl(1,:), 'r');
H4 = plot(time, csurf(1,:), 'k:');
hold off
xlabel('time');
legend([H0,H1,H2,H3,H4], 'acc', 'ang vel', 'AoA', 'pitch c', 'pitch csurf');
h = gca;
set(h, 'Position', [0.045 0.08 0.91 0.90]);
%% Plot roll
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