classdef ang_vel_roll < ang_vel_controller
    
    properties (SetAccess = public)
        max_v_construction = 3.0;
        max_input_v = 1.0;
        min_input_v = -1.0;        
        
        % get_desired_acc section
        quadr_Kp = 0.45;
        relaxation_k = -1.0;
        relaxation_Kp = 0.5;
        relax_count = 0;
        
        already_preupdated = false;
    end
    
    methods (Access = public)
        function c = ang_vel_roll(ac)
            c@ang_vel_controller(ac);
            c.axis = 1;
        end
        
        function cntrl = eval(obj, target, target_acc, dt, yaw_ctl)
            obj.target_vel = target;
            obj.update_pars();
            obj.output_acc = obj.get_desired_acc(target, dt);
            cntrl = obj.acc_c.eval(obj.output_acc + target_acc, yaw_ctl, dt);
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
            
            A = obj.model.roll_A;
            B = obj.model.roll_B;
            C = obj.model.roll_C;
            
            cur_aoa = obj.model.aoa(2);
            if (abs(cur_aoa) < 0.3)
                obj.max_input_v = -(C(1, 1) + B(1, 1) + A(1, 2) + A(1, 3)) / A(1, 1);
                obj.min_input_v = -(C(1, 1) - B(1, 1) - A(1, 2) - A(1, 3)) / A(1, 1);
            else
                obj.max_input_v = obj.max_v_construction;
                obj.min_input_v = obj.min_v_construction;
            end
        end
        
        function des_acc = get_desired_acc(obj, des_v, dt)
            A = obj.model.roll_A;
            B = obj.model.roll_B;
            C = obj.model.roll_C;
            
            if (obj.model.aero_model) %if FAR
                kacc_quadr = obj.quadr_Kp * (A(1, 2) * B(2, 1) + A(1, 3) * B(3, 1) + B(1, 1));
            else
                kacc_quadr = obj.quadr_Kp * (A(1, 2) * C(2, 1) + A(1, 3) * B(3, 1) + B(1, 1));
            end
            kacc_quadr = abs(kacc_quadr);
            cur_v = obj.model.angular_vel(obj.axis + 1);
            v_error = cur_v - des_v;
            quadr_x = -sqrt(abs(v_error) / kacc_quadr);
            if (quadr_x >= - obj.relaxation_k * dt)
                desired_deriv = obj.relaxation_Kp * (-v_error) / (dt * ceil(obj.relaxation_k));
            else
                leftover_dt = min(dt, -quadr_x);
                desired_deriv = sign(v_error) * (kacc_quadr * (quadr_x + leftover_dt) ^ 2 - kacc_quadr * quadr_x^2) / dt;
            end
            des_acc = desired_deriv;
        end
    end
    
end

