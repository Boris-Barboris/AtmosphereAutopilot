classdef aoa_controller < handle
    
    properties (SetAccess = public)
        model;                      % aircraft_model instance
        vel_c;                      % corresponding angular velocity controller
        axis = 0;                   % 0 - pitch 1 - roll 2 - yaw
        output_vel = 0.0;
        output_acc = 0.0;
        user_controlled = false;    % true when target is user control
        
        target_aoa = 0.0;
        cur_aoa_equilibr = 0.0;     % equilibrium angular velocity to stay on current AoA
        
        params = [1.0, 0.0, 0.0];        % math parameters
        
        already_preupdated = false;
    end
    
    methods (Access = public)
        function c = aoa_controller(axis, vc)
            c.vel_c = vc;
            c.model = vc.model;
            c.axis = axis;
        end
        
        function cntrl = eval(obj, target_aoa, target_vel, dt)
            obj.vel_c.preupdate();
            obj.update_pars();
            [obj.output_vel, obj.output_acc] = obj.get_desired_output(target_aoa, dt);
            cntrl = obj.vel_c.eval(obj.output_vel + target_vel, obj.output_acc, dt);
        end
        
        function preupdate(obj)
            obj.update_pars()
            obj.already_preupdated = true;
        end
    end
    
    methods (Access = private)
        
        function update_pars(obj)
            if (obj.already_preupdated)
                obj.already_preupdated = false;
                return
            end
            
            % internal math update
            % let's find cur_aoa_equilibr
            if (obj.axis == 0)
                A = obj.model.pitch_A; 
                B = obj.model.pitch_B;
                C = obj.model.pitch_C;
            else
                A = obj.model.yaw_A; 
                B = obj.model.yaw_B;
                C = obj.model.yaw_C;
            end            
            
            cur_aoa = obj.model.aoa(obj.axis + 1);
            eq_A = zeros(2);
            eq_B = zeros(2, 1);
            eq_A(1, 1) = A(1, 2);
            eq_A(1, 2) = A(1, 3) + A(1, 4) + B(1, 1);
            eq_A(2, 1) = A(2, 2);
            eq_A(2, 2) = A(2, 3) + B(2, 1) + A(2, 4);
            eq_B(1, 1) = -(A(1, 1) * cur_aoa + C(1, 1));
            eq_B(2, 1) = -(A(2, 1) * cur_aoa + C(2, 1));
            eq_x = eq_A \ eq_B;
            obj.cur_aoa_equilibr = eq_x(1, 1);
        end
        
        function [out_vel, out_acc] = get_desired_output(obj, target_aoa, dt)
            cur_aoa = obj.model.aoa(obj.axis + 1);
            obj.target_aoa = aircraft_model.clamp(target_aoa, obj.vel_c.res_min_aoa, obj.vel_c.res_max_aoa);
            
            error = target_aoa - cur_aoa;
            
            output = obj.get_output(error);
            cur_ang_vel = obj.model.angular_vel(obj.axis + 1);
            shift_ang_vel = cur_ang_vel - obj.cur_aoa_equilibr;
            if (shift_ang_vel * error > 0.0)
                predicted_error = error - shift_ang_vel * dt;
                factor = min(shift_ang_vel / output, 1.0) ^ 2;
                out_acc = factor * (obj.get_output(predicted_error) - output) / dt;
            else
                out_acc = 0.0;
            end
            out_vel = obj.cur_aoa_equilibr + output;
        end
        
        function output = get_output(obj, error)
            output = obj.params(1, 1) * error + sign(error) * obj.params(1, 2) * error ^ 2;
            output = output + obj.params(1, 3) * error ^ 3;
        end
        
    end
    
end

