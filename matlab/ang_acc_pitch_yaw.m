classdef ang_acc_pitch_yaw < ang_acc_controller
    
    methods (Access = public)
        function c = ang_acc_pitch_yaw(ax, mod)
            c@ang_acc_controller(mod);
            c.axis = ax;
        end
        
        function cntrl = eval(obj, target, dt)
            obj.target_acc = target;
            
            if (obj.axis == 0)
                B = obj.model.pitch_B;
                Bu = obj.model.pitch_B_undelayed;
                A = obj.model.pitch_A;
                Au = obj.model.pitch_A_undelayed;
                C = obj.model.pitch_C;
            else
                B = obj.model.yaw_B;
                Bu = obj.model.yaw_B_undelayed;
                A = obj.model.yaw_A;
                Au = obj.model.yaw_A_undelayed;
                C = obj.model.yaw_C;
            end
            
            if (~obj.model.aero_model)
                % stock aero                
                Cu = C(1:3);
                % get model prediction for next frame with same input as current csurf
                cur_input = obj.model.csurf_state(obj.axis + 1);
                cur_state = [obj.model.aoa(obj.axis + 1), obj.model.angular_vel(obj.axis + 1), 0.0].';
                out = Au * cur_state + Bu * cur_input + Cu;
                obj.predicted_acc = out(2);
                acc_error = target - obj.predicted_acc;
                authority = Bu(2, 1);
                new_output = aircraft_model.clamp(cur_input + acc_error / authority, -1.0, 1.0);
                % csurf speed checks
                if (abs(new_output - cur_input) / dt > obj.model.stock_csurf_spd)
                    % faster than max speed, delayed mode
                    cur_state = [obj.model.aoa(obj.axis + 1), obj.model.angular_vel(obj.axis + 1),...
                        cur_input, 0.0].';
                    cur_state(3) = aircraft_model.clamp(...
                        cur_state(3) + sign(new_output - cur_input) * ...
                        obj.model.stock_csurf_spd * dt, -1.0, 1.0);
                    cur_input = cur_state(3);
                    out = A * cur_state + B * cur_input + C;
                    obj.predicted_acc = out(2);
                    acc_error = target - obj.predicted_acc;
                    authority = B(2, 1);
                    new_output = aircraft_model.clamp(cur_input + acc_error / authority, -1.0, 1.0);
                end
                obj.predicted_acc = obj.predicted_acc + authority * (new_output - cur_input);
                obj.output_control = new_output;
                cntrl = new_output;
            else
                % FAR aero
                % get model prediction for next frame with same input as current csurf
                cur_input = obj.model.csurf_state(obj.axis + 1);
                cur_state = [obj.model.aoa(obj.axis + 1), obj.model.angular_vel(obj.axis + 1), cur_input, 0.0].';
                out = A * cur_state + B * cur_input + C;
                obj.predicted_acc = out(2);
                acc_error = target - obj.predicted_acc;
                authority = B(2, 1);
                new_output = aircraft_model.clamp(cur_input + acc_error / authority, -1.0, 1.0);
                % csurf far collapse check
                if (abs(new_output - cur_input) * 10 < 0.1)
                    % we collapse instantly to desired state, undelayed mode
                    authority = Bu(2, 1);
                    new_output = aircraft_model.clamp(cur_input + acc_error / authority, -1.0, 1.0);
                end
                obj.predicted_acc = obj.predicted_acc + authority * (new_output - cur_input);
                obj.output_control = new_output;
                cntrl = new_output;
            end
        end
    end
    
end

