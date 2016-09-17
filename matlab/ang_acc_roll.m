classdef ang_acc_roll < ang_acc_controller
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    methods (Access = public)
        function c = ang_acc_roll(mod)
            c@ang_acc_controller(mod);
            c.axis = 1;
        end
        
        function cntrl = eval(obj, target, yaw_cntrl, dt)
            obj.target_acc = target;
            B = obj.model.roll_B;
            Bu = obj.model.roll_B_undelayed;
            A = obj.model.roll_A;
            Au = obj.model.roll_A_undelayed;
            C = obj.model.roll_C;            
            
            if (~obj.model.aero_model)
                % stock aero
                Cu = C(1:2);
                yaw_csurf = aircraft_model.moveto(obj.model.csurf_state(3), ...
                    yaw_cntrl, dt * obj.model.stock_csurf_spd);
                % get model prediction for next frame with same input as current csurf
                cur_input = [obj.model.csurf_state(2), yaw_csurf, obj.model.aoa(3)].';
                cur_state = [obj.model.angular_vel(2), 0.0].';
                out = Au * cur_state + Bu * cur_input + Cu;
                obj.predicted_acc = out(1);
                acc_error = target - obj.predicted_acc;
                authority = Bu(1, 1);
                new_output = aircraft_model.clamp(cur_input(1) + acc_error / authority, -1.0, 1.0);
                % csurf speed checks
                if (abs(new_output - cur_input) / dt > obj.model.stock_csurf_spd)
                    % faster than max speed, delayed mode
                    cur_state = [obj.model.angular_vel(2), cur_input(1), 0.0].';
                    cur_state(2) = aircraft_model.clamp(...
                        cur_state(2) + sign(new_output - cur_input(1)) * ...
                        obj.model.stock_csurf_spd * dt, -1.0, 1.0);
                    cur_input(1) = cur_state(2);
                    out = A * cur_state + B * cur_input + C;
                    obj.predicted_acc = out(1);
                    acc_error = target - obj.predicted_acc;
                    authority = B(1, 1);
                    new_output = aircraft_model.clamp(cur_state(2) + acc_error / authority, -1.0, 1.0);
                end
                obj.predicted_acc = obj.predicted_acc + authority * (new_output - cur_input(1));
                obj.output_control = new_output;
                cntrl = new_output;
            else
                % FAR aero
                yaw_csurf = aircraft_model.moveto_far(obj.model.csurf_state(3), yaw_cntrl, dt);
                % get model prediction for next frame with same input as current csurf
                cur_state = [obj.model.angular_vel(2), obj.model.csurf_state(2), 0.0].';
                cur_input = [obj.model.csurf_state(2), yaw_csurf, obj.model.aoa(3)].';                
                out = A * cur_state + B * cur_input + C;
                obj.predicted_acc = out(1);
                acc_error = target - obj.predicted_acc;
                authority = B(1, 1);
                new_output = aircraft_model.clamp(cur_input(1) + acc_error / authority, -1.0, 1.0);
                % csurf collapse check
                if (abs(new_output - cur_input) * 10 < 0.1)
                    % we're collapsing
                    authority = Bu(1, 1);
                    new_output = aircraft_model.clamp(cur_state(2) + acc_error / authority, -1.0, 1.0);
                end
                obj.predicted_acc = obj.predicted_acc + authority * (new_output - cur_input(1));
                obj.output_control = new_output;
                cntrl = new_output;
            end
        end
    end
    
end

