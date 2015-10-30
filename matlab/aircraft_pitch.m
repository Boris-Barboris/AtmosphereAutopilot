classdef aircraft_pitch
    %AIRCRAFT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        torque_model = [-0.01, 5.0, 11.0];
        lift_model = [0.05, 150.0, 30.0];
        sas_torque = 1.0;
        engine_thrust = 100.0;
        engine_torque_k = 5.0;
        mass = 100.0;
        moi = 40.0;
        far_aero = false;
    end
    
    methods
        function new_state = physics_step(AP, state, input, dt)
            new_state = zeros(3, 1);
            if AP.far_aero
                error = input - state(3);
                if error * 10.0 < 0.1
                    new_state(3) = input;
                else
                    new_state(3) = state(3) + aircraft_pitch.clamp(max(error * dt / 0.25), abs(0.6 * error));
                end
            else
                error = input - state(3);
                new_state(3) = state(3) + aircraft_pitch.clamp(error, 2.0 * dt);
            end
            
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

