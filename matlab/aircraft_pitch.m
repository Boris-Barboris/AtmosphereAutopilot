classdef aircraft_pitch
    %AIRCRAFT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        torque_model = [-0.01, 5.0, 11.0];
        lift_model = [0.05, 150.0, 30.0];
        sas_torque = 10.0;
        engine_thrust = 5000.0;
        engine_torque_k = 100.0;
        mass = 1000.0;
        moi = 100.0;
        far_aero = false;
        surf_speed = 200.0;
    end
    
    methods
        function new_state = physics_step(AP, state, params)
            input = params(1);
            g_projection = -10.0 * cos(state(4) + state(1));
            dt = params(2);
            new_state = zeros(4, 1);
            if AP.far_aero
                error = input - state(3);
                if error * 10.0 < 0.1
                    new_state(3) = input;
                else
                    new_state(3) = state(3) + aircraft_pitch.clamp(error * dt / 0.25,...
                        -0.6 * error, 0.6 * error);
                end
            else
                error = input - state(3);
                new_state(3) = state(3) + aircraft_pitch.clamp(error, -2.0 * dt, 2.0 * dt);
            end
            new_state(1) = state(1) + (( -(AP.engine_thrust * state(1) +...
                AP.lift_model(1) + AP.lift_model(2) * state(1) + AP.lift_model(3) * new_state(3))...
                / AP.mass - g_projection) / AP.surf_speed + state(2)) * dt;
            new_state(2) = state(2) + (AP.torque_model(1) + AP.torque_model(2) * state(1) +...
                AP.torque_model(3) * new_state(3) + input * (AP.sas_torque + AP.engine_torque_k))...
                / AP.moi * dt;
            new_state(4) = state(4) + new_state(2) * dt;
        end
    end
    
    methods (Static)
        function res = clamp(val, low, high)
            if val > high
                res = high;
            else
                if val < low
                    res = low;
                else
                    res = val;
                end
            end
        end
        
    end
    
end

