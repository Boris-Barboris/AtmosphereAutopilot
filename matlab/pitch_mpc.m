%% initialize plane characteristics
moi = 100.0;
mass = 200.0;
g = 10.0;
gravity = mass * g;
density = 1.0;
airspd = 200.0;
dyn_pressure = density * airspd^2;

% Calculate lift coefficient
horizontal_flight_aoa = 0.05;
% Lift == gravity
% aoa * Cl == gravity
% Cl = gravity / aoa
Cl = gravity / horizontal_flight_aoa;

% set control lag factor
csurf_exp_factor = 0.25;

% linear control torques
sas_torque = 5.0;
engine_torque = 5.0;

% control and aoa aothority
k0 = 0.0;
k1 = -100.0;
k2 = 800.0;

% limits
max_aoa = 0.25;
max_v = 0.5;
max_g = 100.0;

dt = 0.025;
% model evolution law:
% x' = A * x + B * u + C
% x(i+1) = x(i) + (A*x(i) + B*u(i) + C)*dt
A = zeros(3);
B = zeros(3, 1);
C = zeros(3, 1);
%% run simulation
init_aoa = -0.25;
init_ang_vel = 0.2;
init_csurf = 0.0;
init_pitch = 0.0;

x = zeros(3,1);                 % current state-vector, zero it out
x(1) = init_aoa;
x(2) = init_ang_vel;
x(3) = init_csurf;
cur_pitch = init_pitch;         % non-linear pitch angle

% evaluate initial transfer matrixes
A(1,1) = - Cl / mass / airspd;
A(1,2) = 1.0;
A(2,1) = k1 / moi;
A(2,3) = k2 * (1.0 - dt / csurf_exp_factor) / moi;
A(3,3) = -1.0 / csurf_exp_factor;
B(2,1) = (sas_torque + dt / csurf_exp_factor) / moi;
B(3,1) = 1.0 / csurf_exp_factor;
C(1,1) = g * cos(cur_pitch) / airspd;
C(2,1) = k0 / moi;
% save them for MPC
Ai = A;
Bi = B;
Ci = C;

% user meta-variables
u = init_csurf;                     % current input vector
desired_v = 0.0;

simul_time = 15.0;
simul_length = int16(simul_time / dt);

% prepare output vectors
time = linspace(0.0, double(simul_length) * dt, simul_length + 1);
aoa = zeros(1, simul_length + 1);
ang_vel = zeros(1, simul_length + 1);
csurf = zeros(1, simul_length + 1);
input = zeros(1, simul_length + 1);
pitch = zeros(1, simul_length + 1);
desired_ang_v_arr = zeros(1, simul_length + 1);
mpc_predict = zeros(1, simul_length + 1);
mpc_predict(1) = init_ang_vel;

% Get moderation parameters
res_max_aoa = max_aoa;
res_max_v = max_v;
% find balanced state for steady turn on 1.0 input
% it is also a maximum controllable aoa
steady_input_turn_x = max(Ai \ (- Bi .* 1.0 - Ci), Ai \ (Bi .* 1.0 - Ci));
res_max_aoa = min(res_max_aoa, abs(steady_input_turn_x(1)));
res_max_v = min(res_max_v, abs(steady_input_turn_x(2)));
% find angular velocity on steady turn with max_g g-force
steady_g_turn_v = max_g / airspd;
steady_g_turn_aoa = (max_g + g * cos(cur_pitch)) * mass / Cl;
res_max_aoa = min(res_max_aoa, abs(steady_g_turn_aoa));
res_max_v = min(res_max_v, steady_g_turn_v);
% find balanced angular velocity for steady turn on res_max_aoa
steady_aoa_turn_v = (Cl * res_max_aoa / mass - g * cos(cur_pitch)) / airspd;
res_max_v = min(res_max_v, steady_aoa_turn_v);

% Controller gains
Kacc = 8.0;

% let's try naive value of dyn_max_v
dyn_max_v = 0.5 * sqrt(res_max_aoa * (k0 + k1 * res_max_aoa / 2.0 + k2 + sas_torque) / moi);
C(1,1) = 0.0;

mpc_dt = 0.025;
v_eps = 1e-2;
% tune dyn_max_v with relaxation simulation
while false
    % perform simulation
    x_sim = [res_max_aoa; -dyn_max_v; 0.0];     % start on max_aoa with minimum angvel
    u_sim = 0.0;
    under_zero = 0;
    iter_limit = 1000;   % 10 seconds simulation frame
    iter = 0;
    cur_pitch_sim = cur_pitch;
    while true
        aoa(iter+1) = x_sim(1);
        ang_vel(iter+1) = x_sim(2);
        csurf(iter+1) = x_sim(3);
        input(iter+1) = u_sim;
        
        desired_acc = -Kacc * x_sim(2);     % relaxate to v = 0.0
        % get predicted state derivative
        pr_dx = Ai * x_sim + Bi * u_sim + Ci;
        cntrl_auth = Bi(2);     % supposed authority of control
        pr_acc = pr_dx(2);      % predicted acc
        acc_err = desired_acc - pr_acc;
        acc_sat = 1.0;          % how fast we want to converge to desired_acc
        new_u = max(-1.0, min(1.0, u_sim + acc_err / cntrl_auth * acc_sat));
        x_sim = x_sim + (Ai * x_sim + Bi * new_u + Ci) .* mpc_dt;    % predicted x
        % apply control
        u_sim = new_u;

        % nonlinear pitch transfer
        cur_pitch_sim = cur_pitch_sim + (Cl * x_sim(1) / mass - g * cos(cur_pitch_sim)) / airspd * mpc_dt;
        
        % check if we finished
        if abs(x_sim(2)) < v_eps
            under_zero = under_zero + 1;
            if under_zero > 10
                break; 
            end
        end
        
        if abs(x_sim(1)) > res_max_aoa
            break;
        end
        
        iter = iter+1;
        if iter > iter_limit
            break; 
        end
    end
    % decrease dyn_max_v if needed
    if (x_sim(2) > v_eps) || (abs(x_sim(1)) > res_max_aoa)
        dyn_max_v = 0.9 * dyn_max_v;
        if dyn_max_v < 0.01
            break;
        end
    else
        break;
    end
end

aoa(1) = init_aoa;
ang_vel(1) = init_ang_vel;
csurf(1) = init_csurf;
input(1) = u;
desired_ang_v_arr(1) = desired_v;

% Acc control section
dx_bias = zeros(3,1);
first_cycle = true;

% bias of moder error
%k0 = k0 + 0.01;
%k1 = k1 - 100.0;
%k2 = k2 - 200.0;

% do simulate
for frame = 2:simul_length+1
    % noise
    k0_noise = 0.0 * (rand(1) - 0.5) * 0.005;
    k1_noise = 0.0 * (rand(1) - 0.5) * 10.0;
    k2_noise = 0.0 * (rand(1) - 0.5) * 20.0;
    Cl_moise = 0.0 * (rand(1) - 0.5) * 200.0;
    % update transfer matrixes
    A(1,1) = - (Cl + Cl_moise) / mass / airspd;
    A(1,2) = 1.0;
    A(2,1) = (k1 + k1_noise) / moi;
    A(2,3) = (k2 + k2_noise) * (1.0 - dt / csurf_exp_factor) / moi;
    A(3,3) = -1.0 / csurf_exp_factor;
    B(2,1) = (sas_torque + (k2 + k2_noise) * dt / csurf_exp_factor) / moi;
    B(3,1) = 1.0 / csurf_exp_factor;
    C(1,1) = g * cos(cur_pitch) / airspd;
    C(2,1) = (k0 + k0_noise) / moi;
    
    if double(frame) * dt > 7.0
        desired_v = 0.0;
    end
    
    % ANGULAR VEL Controller
    
    % update moderation values
    res_max_aoa = max_aoa;
    res_max_v = max_v;
    res_max_aoa = min(res_max_aoa, abs(steady_input_turn_x(1)));
    res_max_v = min(res_max_v, abs(steady_input_turn_x(2)));
    % find angular velocity on steady turn with max_g g-force
    steady_g_turn_aoa = max_g * mass / Cl;
    res_max_aoa = min(res_max_aoa, abs(steady_g_turn_aoa));
    res_max_v = min(res_max_v, steady_g_turn_v);
    % find balanced angular velocity for steady turn on res_max_aoa
    steady_aoa_turn_v = (Cl * res_max_aoa / mass - g * cos(cur_pitch)) / airspd;
    res_max_v = min(res_max_v, steady_aoa_turn_v);
    
    % linear scaling of v limit
    if desired_v - x(1) >= 0.0
        scaled_aoa = (res_max_aoa - x(1)) / (2.0 * res_max_aoa);
        dyn_desired_v = min(dyn_max_v, desired_v);
        scaled_restrained_v = min(dyn_desired_v,...
            dyn_desired_v * scaled_aoa + res_max_v * (1.0 - scaled_aoa));
    else
        scaled_aoa = (x(1) + res_max_aoa) / (2.0 * res_max_aoa);
        dyn_desired_v = max(-dyn_max_v, desired_v);
        scaled_restrained_v = max(dyn_desired_v,...
            dyn_desired_v * scaled_aoa - res_max_v * (1.0 - scaled_aoa));
    end    
    v_error = x(2) - scaled_restrained_v;
    % let's descend by quadratic function
    kacc_quadr = 0.2 * A(2,3) * B(3,1);
    %kacc_quadr = 0.1;
    if v_error >= 0.0
        quadr_x = -sqrt(v_error / kacc_quadr);
        desired_deriv = (kacc_quadr * (quadr_x + dt)^2 - kacc_quadr * quadr_x^2) / dt;
    else
        quadr_x = -sqrt(v_error / -kacc_quadr);
        %desired_deriv = -quadr_x * 2.0 * kacc_quadr;
        desired_deriv = (-kacc_quadr * (quadr_x + dt)^2 + kacc_quadr * quadr_x^2) / dt;
    end
    if abs(quadr_x) <= dt
        desired_deriv = -v_error / dt;
    end
    %Kacc = sqrt(abs(B(2, 1) / v_error));
    %desired_acc = -Kacc * v_error;
    desired_acc = desired_deriv;
    
    % ANGULAR ACC Controller
    
    % analyze errors in model
    if ~first_cycle
        dx_bias = dx - pr_x;   % how big was x' prediction error
    end
    first_cycle = false;
    % get predicted state derivative
    pr_dx = A * x + B * u + Ci;
    cntrl_auth = B(2);     % supposed authority of control
    pr_acc = pr_dx(2);      % predicted acc
    acc_err = desired_acc - pr_acc;
    acc_sat = 1.0;          % how fast we want to converge to desired_acc
    new_u = max(-1.0, min(1.0, u + acc_err / cntrl_auth * acc_sat));
    delta_u = new_u - u;
    pr_x = x + (A * x + B * new_u + C) .* dt;    % predicted x
    pr_dx = pr_dx + delta_u * cntrl_auth;
    % apply control
    u = new_u;
    
    % do calculations
    dx = (A * x + B * u + C) .* dt;
    x = x + dx;
    % nonlinear pitch transfer
    cur_pitch = cur_pitch + (Cl * x(1) / mass - g * cos(cur_pitch)) / airspd * dt;
    
    % write to outputs
    aoa(frame) = x(1);
    ang_vel(frame) = x(2);
    csurf(frame) = x(3);
    input(frame) = u;
    pitch(frame) = cur_pitch;
    desired_ang_v_arr(frame) = scaled_restrained_v;
end

%% plot outputs
scrsz = get(0,'ScreenSize');
figure('Name','Pitch simulation',...
    'Position',[100 50 scrsz(3)*0.9 scrsz(4)*0.8])
plot(time, aoa, 'r');
hold on
plot(time, ang_vel, 'b');
plot(time, desired_ang_v_arr, 'b--');
plot(time, csurf, 'k:');
plot(time, input, 'k');
plot(time, pitch, 'm');
hold off;
xlabel('time')
legend('aoa', 'vel', 'moderated desired vel', 'csurf', 'input', 'airspeed pitch');
h = gca;
set(h, 'Position', [0.035 0.06 0.96 0.92]);