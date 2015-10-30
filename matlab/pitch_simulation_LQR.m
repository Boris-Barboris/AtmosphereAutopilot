%% initialize plane characteristics
moi = 100.0;
mass = 200.0;
g = 10.0;
gravity = mass * g;
density = 1.0;
airspd = 200.0;
dyn_pressure = density * airspd^2;

% Calculate lift coefficient
horizontal_flight_aoa = 0.07;
% Lift == gravity
% aoa * Cl == gravity
% Cl = gravity / aoa
Cl = gravity / horizontal_flight_aoa;

% set control lag factor
csurf_exp_factor = 0.25;

% linear control torques
sas_torque = 10.0;
engine_torque = 10.0;

% control and aoa aothority
k0 = 0.0;
k1 = -500.0;
k2 = 150.0;

% limits
max_aoa = 0.25;
max_v = 1.0;

dt = 0.025;
% model evolution law:
% x(i+1) = A * x(i) + B * u(i)
A = zeros(4);
B = zeros(4, 1);
% create transfer matrixes
A(1,1) = 1.0 - Cl * dt / mass / airspd;
A(1,2) = dt;
A(2,1) = k1 * dt / moi;
A(2,2) = 1.0;
A(2,3) = k2 * dt / moi;
A(2,4) = k0 * dt / moi;
A(3,3) = 1.0 - dt / csurf_exp_factor;
A(4,4) = 1.0;
B(2,1) = sas_torque * dt / moi + k2 * dt / moi * dt / csurf_exp_factor;
B(3,1) = dt / csurf_exp_factor;

% prepare performance matrixes
q_weights = [0.0, 10.0 / max_v^2, 0.0, 0.0];
C = diag(q_weights);
Q = C.' * C;
R = 0.01 * eye(1);
%% run simulation
init_aoa = 0.2;
init_ang_vel = 0.0;
init_csurf = 0.0;
init_pitch = 0.0;

x = zeros(4,1);     % current state-vector, zero it out
x(1) = init_aoa;
x(2) = init_ang_vel;
x(3) = init_csurf;
x(4) = 1.0;
cur_pitch = init_pitch;             % non-linear pitch angle
A(1,4) = g * cos(cur_pitch) * dt / airspd;

% user meta-variables
u = init_csurf;                     % current input vector
desired_v = 0.0;

simul_time = 5.0;
simul_length = int16(simul_time / dt);

% prepare output vectors
time = linspace(0.0, double(simul_length) * dt, simul_length + 1);
aoa = zeros(1, simul_length + 1);
aoa(1) = init_aoa;
ang_vel = zeros(1, simul_length + 1);
ang_vel(1) = init_ang_vel;
csurf = zeros(1, simul_length + 1);
csurf(1) = init_csurf;
input = zeros(1, simul_length + 1);
input(1) = u;
pitch = zeros(1, simul_length + 1);
pitch(1) = init_pitch;

% generate controller
p_iter = int16(30);
P = Q;
for i = 1:p_iter
    P = Q + A.' * P * A - A.' * P * B * inv(R + B.' * P * B) * B.' * P * A;
end
K = inv(R + B.' * P * B) * B.' * P * A;

% do simulate
for frame = 2:simul_length+1
    % noise
    k0_noise = (rand(1) - 0.5) * 0.01;
    k1_noise = (rand(1) - 0.5) * 5.0;
    k2_noise = (rand(1) - 0.5) * 3.0;
    % update transfer matrixes
    A(1,1) = 1.0 - Cl * dt / mass / airspd;
    A(1,2) = dt;
    A(1,4) = g * cos(cur_pitch) * dt / airspd;
    A(2,1) = (k1 + k1_noise) * dt / moi;
    A(2,2) = 1.0;
    A(2,3) = (k2 + k2_noise) * dt / moi;
    A(2,4) = (k0 + k0_noise) * dt / moi;
    A(3,3) = 1.0 - dt / csurf_exp_factor;
    A(4,4) = 1.0;
    B(2,1) = sas_torque * dt / moi + k2 * dt / moi * dt / csurf_exp_factor;
    B(3,1) = dt / csurf_exp_factor;
    
    % get delta state vector
    desired = [0.0; desired_v; 0.0; 1.0];
    x_error = desired - x;
    
    % do calculations
    u = - K * x;
    u = max(min(u, 1.0), -1.0);
    x = A * x + B * u;
    % nonlinear pitch transfer
    cur_pitch = cur_pitch + (Cl * x(1) / mass - g * cos(cur_pitch)) / airspd * dt;
    
    % write to outputs
    aoa(frame) = x(1);
    ang_vel(frame) = x(2);
    csurf(frame) = x(3);
    input(frame) = u;
    pitch(frame) = cur_pitch;
end

%% plot outputs
scrsz = get(0,'ScreenSize');
figure('Name','Pitch simulation',...
    'Position',[100 50 scrsz(3)*0.9 scrsz(4)*0.8])
plot(time, aoa, 'r');
hold on
plot(time, ang_vel, 'b');
plot(time, csurf, 'k');
plot(time, input, 'k:');
plot(time, pitch, 'm');
hold off;
xlabel('time')
legend('aoa', 'vel', 'csurf', 'input', 'airspeed pitch');
h = gca;
set(h, 'Position', [0.035 0.06 0.96 0.92]);