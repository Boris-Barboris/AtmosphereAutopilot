AP = aircraft_pitch;
AP.torque_model = [-0.3, -100.0, 50.0];
AP.lift_model = [0.05, 150000.0, 3000.0];
AP.sas_torque = 1.0;
AP.engine_thrust = 5000.0;
AP.engine_torque_k = 50.0;
AP.mass = 1000.0;
AP.moi = 500.0;
AP.far_aero = false;
AP.surf_speed = 200.0;

max_aoa = 0.2;
dt = 0.025;
t = 0.0;
t_max = 3.0;
t_intervals = int16(t_max / dt + 1);

state = [0.0; 0.4; -1.0; 0.0];

t_vector = zeros(1, t_intervals);
states_vector = zeros(4, t_intervals);
input_vector = zeros(1, t_intervals);

%% iteration itself

for i = 1:t_intervals
    t_vector(i) = t;
    states_vector(:,i) = state;
    cur_step_input = -1.0;
    input_vector(i) = cur_step_input;
    
    state = AP.physics_step(state, [cur_step_input, dt]);
    t = t + dt;
end

%% plotting

scrsz = get(0,'ScreenSize');
figure('Name','Pitch simulation',...
    'Position',[100 50 scrsz(3)*0.9 scrsz(4)*0.8])
plot(t_vector, states_vector(1,:), 'r');
hold on
plot(t_vector, states_vector(2,:), 'b');
plot(t_vector, states_vector(3,:), 'k:');
plot(t_vector, input_vector, 'k');
plot(t_vector, states_vector(4,:), 'm');
plot([0.0, t_max], [max_aoa, max_aoa], 'r:');
xlabel('time')
legend('aoa', 'vel', 'csurf', 'input', 'pitch angle', 'aoa limit');
hold off;
h = gca;
set(h, 'Position', [0.035 0.05 0.96 0.91]);