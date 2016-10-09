classdef aircraft_model < handle
    %AIRCRAFT_MODEL Linear plane model for control system development
    
    properties (SetAccess = public)        
        %
        % state definitions
        %
        position = [0.0 0.0 500.0];      % height will be 500 m for start
        rotation = quaternion([1,0,0,0]);
        right_vector = [1.0 0.0 0.0];
        forward_vector = [0.0 1.0 0.0];
        up_vector = zeros(1, 3);
        velocity = [0.0 200.0 0.0];      % flying forward initially
        velocity_magn = 0.0;             % velocity magnitude
        
        angular_vel = [0.0 0.0 0.0];     % angular vel vector
        angular_acc = [0.0 0.0 0.0];     % angular acc vector
        csurf_state = [0.0 0.0 0.0];     % control surfaces position
        csurf_state_new = [0.0 0.0 0.0];     % control surfaces FAR service buffer
        gimbal_state = [0.0 0.0 0.0];    % gimbal directions
        aoa = [0.0 0.0 0.0];             % aoas
        
        % environment definition
        density = 1.0;                   % density
        dyn_pressure = 0.0;
        pitch_gravity_acc = -9.8;
        yaw_gravity_acc = 0.0;
        
        pitch_tangent = [0.0 0.0 1.0];
        yaw_tangent = [1.0 0.0 0.0];
        
        %
        % model definitions
        %
        aero_model = false;              % false for stock, true for FAR
        stock_csurf_spd = 2.0;           % stock csurf speed
        far_timeConstant = 0.25;         % default FAR csurf timeConstant
        MOI = [165.0 25.0 180.0];
        mass = 14.0;
        
        % aero models
        pitch_rot_m = [0.0 -1.0 1.15];   % pitch rotation model
        roll_rot_m = [0.0 0.6 -0.35 0.6 -0.28];    % roll rotation model
        yaw_rot_m = [0.0 -2.0 0.8];      % yaw rotation model
        pitch_lift_m = [0.0 60.0 -0.25]; % pitch lift model
        yaw_lift_m = [0.0 8.0 -2.5];     % sideslip lift model
        
        drag_model = [1.0 20.0 3.0];     % drag model
        force_spd_maintain = false;      % forcefully maintain speed for debug purposes
        
        % engine power
        
        % rcs and reaction wheels
        sas_torque = [15.0 15.0 15.0];
        
        %
        % Exported matrixes
        %
        pitch_A = zeros(4, 4);
        pitch_B = zeros(4, 1);
        pitch_C = zeros(4, 1);
        pitch_A_undelayed = zeros(3, 3);
        pitch_B_undelayed = zeros(3, 1);
        
        roll_A = zeros(3, 3);
        roll_B = zeros(3, 3);
        roll_C = zeros(3, 1);
        roll_A_undelayed = zeros(2, 2);
        roll_B_undelayed = zeros(2, 3);
        
        yaw_A = zeros(4, 4);
        yaw_B = zeros(4, 1);
        yaw_C = zeros(4, 1);
        yaw_A_undelayed = zeros(3, 3);
        yaw_B_undelayed = zeros(3, 1);
    end
    
    methods (Access = public)
        % analogue of PreAutopilotUpdate
        function preupdate(obj, dt)
            % we need to prepare data here
            obj.right_vector = RotateVector(obj.rotation, [1.0; 0.0; 0.0]);
            obj.forward_vector = RotateVector(obj.rotation, [0.0; 1.0; 0.0]);
            obj.up_vector = cross(obj.right_vector, obj.forward_vector);
            obj.velocity_magn = norm(obj.velocity);
            obj.dyn_pressure = obj.density * obj.velocity_magn * obj.velocity_magn;
            update_dynamics(obj);
            update_pitch_matrix(obj, dt);
            update_roll_matrix(obj, dt);
            update_yaw_matrix(obj, dt);
            update_aoa(obj);
        end
        
        % physics simulation
        function simulation_step(obj, dt, input)
            update_control_states(obj, dt, input);
            integrate_dynamics(obj, dt, input);
        end
    end
    
    methods(Static, Access = public)
        function r = clamp(a, l, u)
            r = max(l, min(u, a));
        end
        
        function r = lerp(a, b, t)
            r = a + (b - a) * aircraft_model.clamp(t, 0.0, 1.0);
        end
        
        function r = moveto(a, b, delta_limit)
            r = a + aircraft_model.clamp(b - a, -delta_limit, delta_limit);
        end
        
        function r = moveto_far(a, b, time_div)
            error = b - a;
            if (abs(error) * 10 >= 0.1)
                r = a + aircraft_model.clamp(error * time_div, -abs(0.6 * error), abs(0.6 * error));
            else
                r = b;
            end
        end
        
        function r = project(v, norm)
            r = norm * dot(v, norm);
        end
    end
    
    methods (Access = private)
        function update_control_states(obj, dt, input)
            if (~obj.aero_model)
                obj.csurf_state(1) = aircraft_model.moveto(obj.csurf_state(1), input(1), dt * obj.stock_csurf_spd);
                obj.csurf_state(2) = aircraft_model.moveto(obj.csurf_state(2), input(2), dt * obj.stock_csurf_spd);
                obj.csurf_state(3) = aircraft_model.moveto(obj.csurf_state(3), input(3), dt * obj.stock_csurf_spd);
            else
                obj.csurf_state_new(1) = aircraft_model.moveto_far(obj.csurf_state(1), input(1), dt / obj.far_timeConstant);
                obj.csurf_state_new(2) = aircraft_model.moveto_far(obj.csurf_state(2), input(2), dt / obj.far_timeConstant);
                obj.csurf_state_new(3) = aircraft_model.moveto_far(obj.csurf_state(3), input(3), dt / obj.far_timeConstant);
            end
        end
        
        function integrate_dynamics(obj, dt, input)
            % translation
            acc = [0.0 0.0 -9.8];       % gravity acceleration
            
            dk0 = obj.drag_model(1) * 1e-3 * obj.dyn_pressure / obj.mass;
            dk1 = obj.drag_model(2) * 1e-3 * obj.dyn_pressure / obj.mass;
            dk2 = obj.drag_model(3) * 1e-3 * obj.dyn_pressure / obj.mass;
            drag_acc = - (obj.velocity / obj.velocity_magn * (dk0 + dk1 * obj.aoa(1) * obj.aoa(1) + dk2 * obj.aoa(3) * obj.aoa(3)));
            
            Cl0 = obj.pitch_lift_m(1) * 1e-3 * obj.dyn_pressure / obj.mass;
            Cl1 = obj.pitch_lift_m(2) * 1e-3 * obj.dyn_pressure / obj.mass;
            Cl2 = obj.pitch_lift_m(3) * 1e-3 * obj.dyn_pressure / obj.mass;
            pitch_lift_acc = obj.pitch_tangent * (Cl0 + Cl1 * obj.aoa(1) + Cl2 * obj.csurf_state(1));
            
            Cl0 = obj.yaw_lift_m(1) * 1e-3 * obj.dyn_pressure / obj.mass;
            Cl1 = obj.yaw_lift_m(2) * 1e-3 * obj.dyn_pressure / obj.mass;
            Cl2 = obj.yaw_lift_m(3) * 1e-3 * obj.dyn_pressure / obj.mass;
            yaw_lift_acc = obj.yaw_tangent * (Cl0 + Cl1 * obj.aoa(3) + Cl2 * obj.csurf_state(3));
            
            acc = acc + drag_acc + pitch_lift_acc + yaw_lift_acc;
            obj.position = obj.position + obj.velocity * dt + 0.5 * acc * dt * dt;
            obj.velocity = obj.velocity + acc * dt;
            if (obj.force_spd_maintain)
                % keep speed the same
                obj.velocity = obj.velocity * (obj.velocity_magn / norm(obj.velocity));
            end
            
            % rotation
            pitch_acc = obj.pitch_A * [obj.aoa(1), obj.angular_vel(1), obj.csurf_state(1), 0.0].' +...
                obj.pitch_B * input(1) + obj.pitch_C;
            obj.angular_acc(1) = pitch_acc(2);
            roll_acc = obj.roll_A * [obj.angular_vel(2), obj.csurf_state(2), 0.0].' +...
                obj.roll_B * [input(2), obj.csurf_state(3), obj.aoa(3)].' + obj.roll_C;
            obj.angular_acc(2) = roll_acc(1);
            yaw_acc = obj.yaw_A * [obj.aoa(3), obj.angular_vel(3), obj.csurf_state(3), 0.0].' +...
                obj.yaw_B * input(3) + obj.yaw_C;
            obj.angular_acc(3) = yaw_acc(2);
            
            if (obj.aero_model)
                obj.csurf_state = obj.csurf_state_new;
            end
            
            rot_deltas = obj.angular_vel * dt + 0.5 * dt * dt * obj.angular_acc;
            rot_quat = quaternion.eulerangles('xyz', rot_deltas);
            obj.angular_vel = obj.angular_vel + dt * obj.angular_acc;
            obj.rotation = obj.rotation * rot_quat;
        end
        
        function update_aoa(obj)
            up_surf_v = aircraft_model.project(obj.velocity, obj.forward_vector);
            fwd_surf_v = aircraft_model.project(obj.velocity, -obj.up_vector);
            right_surf_v = aircraft_model.project(obj.velocity, obj.right_vector);
            
            projected_vel = up_surf_v + fwd_surf_v;
            aoa_p = asin(aircraft_model.clamp(dot(-obj.up_vector, projected_vel / norm(projected_vel)), -1.0, 1.0));
            if (dot(projected_vel, obj.forward_vector) < 0.0)
                aoa_p = pi - aoa_p;
            end
            obj.aoa(1) = aoa_p;
            
            projected_vel = up_surf_v + right_surf_v;
            aoa_y = asin(aircraft_model.clamp(dot(obj.right_vector, projected_vel / norm(projected_vel)), -1.0, 1.0));
            if (dot(projected_vel, obj.forward_vector) < 0.0)
                aoa_y = pi - aoa_y;
            end
            obj.aoa(3) = aoa_y;
        end
        
        function update_dynamics(obj)
            surf_v_norm = obj.velocity / norm(obj.velocity);
            % pitch
            obj.pitch_tangent = cross(obj.right_vector, surf_v_norm);
            obj.pitch_tangent = obj.pitch_tangent / norm(obj.pitch_tangent);
            obj.pitch_gravity_acc = dot([0.0 0.0 -9.8], obj.pitch_tangent);
            % yaw
            obj.yaw_tangent = cross(obj.up_vector, surf_v_norm);
            obj.yaw_tangent = obj.yaw_tangent / norm(obj.yaw_tangent);
            obj.yaw_gravity_acc = dot([0.0 0.0 -9.8], obj.yaw_tangent);
        end
        
        function update_pitch_matrix(obj, dt)
            Cl0 = obj.pitch_lift_m(1) * 1e-3 * obj.dyn_pressure / obj.mass;
            Cl1 = obj.pitch_lift_m(2) * 1e-3 * obj.dyn_pressure / obj.mass;
            Cl2 = obj.pitch_lift_m(3) * 1e-3 * obj.dyn_pressure / obj.mass;
            K0 = obj.pitch_rot_m(1) * 1e-2 * obj.dyn_pressure / obj.MOI(1);
            K1 = obj.pitch_rot_m(2) * 1e-2 * obj.dyn_pressure / obj.MOI(1);
            K2 = obj.pitch_rot_m(3) * 1e-2 * obj.dyn_pressure / obj.MOI(1);
            
            obj.pitch_A(1, 1) = - Cl1 / obj.velocity_magn;
            obj.pitch_A(1, 2) = 1.0;
            obj.pitch_A(1, 3) = - Cl2 / obj.velocity_magn;
            obj.pitch_A(2, 1) = K1;
            if (~obj.aero_model)
                obj.pitch_A(2, 3) = K2;
                obj.pitch_B(2, 1) = obj.sas_torque(1) / obj.MOI(1);
                obj.pitch_C(3, 1) = obj.stock_csurf_spd;
            else
                % FAR
                obj.pitch_A(2, 3) = K2 * (1.0 - dt / obj.far_timeConstant);
                obj.pitch_B(2, 1) = obj.sas_torque(1) / obj.MOI(1) + K2 * dt / obj.far_timeConstant;
                obj.pitch_A(3, 3) = -1.0 / obj.far_timeConstant;
                obj.pitch_B(3, 1) = 1.0 / obj.far_timeConstant;
            end
            obj.pitch_C(1, 1) = -(obj.pitch_gravity_acc + Cl0) / obj.velocity_magn;
            obj.pitch_C(2, 1) = K0;
            
            obj.pitch_A_undelayed = obj.pitch_A(1:3,1:3);
            obj.pitch_A_undelayed(2, 3) = 0.0;
            obj.pitch_B_undelayed = obj.pitch_B(1:3);
            obj.pitch_B_undelayed(2, 1) = obj.sas_torque(1) / obj.MOI(1) + K2;          
        end
        
        function update_roll_matrix(obj, dt)
            K0 = obj.roll_rot_m(1) * 1e-2 * obj.dyn_pressure / obj.MOI(2);
            K1 = obj.roll_rot_m(2) * 1e-2 * obj.dyn_pressure / obj.MOI(2);
            K2 = obj.roll_rot_m(3) * obj.dyn_pressure / obj.MOI(2) / obj.velocity_magn;
            K3 = obj.roll_rot_m(4) * 1e-2 * obj.dyn_pressure / obj.MOI(2);
            K4 = obj.roll_rot_m(5) * 1e-2 * obj.dyn_pressure / obj.MOI(2);
            
            obj.roll_A(1, 1) = K2;
            if (~obj.aero_model)
                obj.roll_A(1, 2) = K3;
                obj.roll_B(1, 1) = obj.sas_torque(2) / obj.MOI(2);
                obj.roll_C(2, 1) = obj.stock_csurf_spd;
            else
                obj.roll_A(1, 2) = K3 * (1.0 - dt / obj.far_timeConstant);
                obj.roll_B(1, 1) = obj.sas_torque(2) / obj.MOI(2) + K3 * dt / obj.far_timeConstant;
                obj.roll_A(2, 2) = -1.0 / obj.far_timeConstant;
                obj.roll_B(2, 1) = 1.0 / obj.far_timeConstant;
            end
            obj.roll_B(1, 2) = K4;
            obj.roll_B(1, 3) = K1;
            obj.roll_C(1, 1) = K0;
            
            obj.roll_A_undelayed = obj.roll_A(1:2,1:2);
            obj.roll_A_undelayed(1, 2) = 0.0;
            obj.roll_B_undelayed = obj.roll_B(1:2,1:3);
            obj.roll_B_undelayed(1, 1) = obj.sas_torque(2) / obj.MOI(2) + K3;
        end
        
        function update_yaw_matrix(obj, dt)
            Cl0 = obj.yaw_lift_m(1) * 1e-3 * obj.dyn_pressure / obj.mass;
            Cl1 = obj.yaw_lift_m(2) * 1e-3 * obj.dyn_pressure / obj.mass;
            Cl2 = obj.yaw_lift_m(3) * 1e-3 * obj.dyn_pressure / obj.mass;
            K0 = obj.yaw_rot_m(1) * 1e-2 * obj.dyn_pressure / obj.MOI(3);
            K1 = obj.yaw_rot_m(2) * 1e-2 * obj.dyn_pressure / obj.MOI(3);
            K2 = obj.yaw_rot_m(3) * 1e-2 * obj.dyn_pressure / obj.MOI(3);
            
            obj.yaw_A(1, 1) = - Cl1 / obj.velocity_magn;
            obj.yaw_A(1, 2) = 1.0;
            obj.yaw_A(1, 3) = - Cl2 / obj.velocity_magn;
            obj.yaw_A(2, 1) = K1;
            if (~obj.aero_model)
                obj.yaw_A(2, 3) = K2;
                obj.yaw_B(2, 1) = obj.sas_torque(1) / obj.MOI(3);
                obj.yaw_C(3, 1) = obj.stock_csurf_spd;
            else
                obj.yaw_A(2, 3) = K2 * (1.0 - dt / obj.far_timeConstant);
                obj.yaw_B(2, 1) = obj.sas_torque(3) / obj.MOI(3) + K2 * dt / obj.far_timeConstant;
                obj.yaw_A(3, 3) = -1.0 / obj.far_timeConstant;
                obj.yaw_B(3, 1) = 1.0 / obj.far_timeConstant;
            end
            obj.yaw_C(1, 1) = -(obj.yaw_gravity_acc + Cl0) / obj.velocity_magn;
            obj.yaw_C(2, 1) = K0;
            
            obj.yaw_A_undelayed = obj.yaw_A(1:3,1:3);
            obj.yaw_A_undelayed(2, 3) = 0.0;
            obj.yaw_B_undelayed = obj.yaw_B(1:3);
            obj.yaw_B_undelayed(2, 1) = obj.sas_torque(3) / obj.MOI(3) + K2;           
        end
    end    
end

