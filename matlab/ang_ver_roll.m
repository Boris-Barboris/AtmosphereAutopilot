classdef ang_vel_pitch_yaw < ang_vel_controller
    
    properties (SetAccess = public)
        max_v_construction = 3.0;
    end
    
    methods (Access = public)
        function c = ang_vel_pitch_yaw(ac)
            c@ang_vel_controller(ac);
            c.axis = 1;
        end
        
        function cntrl = eval(obj, target, target_acc, dt)
            obj.target_vel = target;
            obj.update_pars();
            obj.output_acc = obj.get_desired_acc(target, dt);
            cntrl = obj.acc_c.eval(obj.output_acc + target_acc, dt);
        end
    end
    
    methods (Access = private)
        function update_pars(obj)
            
        end
        
        function des_acc = get_desired_acc(obj, des_v, dt)
            if (obj.axis == 0)
                A = obj.model.pitch_A;
                B = obj.model.pitch_B;
                C = obj.model.pitch_C;
            else
                A = obj.model.yaw_A;
                B = obj.model.yaw_B;
                C = obj.model.yaw_C;
            end
            
            if (obj.model.aero_model) %if FAR
                kacc_quadr = obj.quadr_Kp * (A(2, 3) * B(3, 1) + A(2, 4) * C(4, 1) + B(2, 1));
            else
                kacc_quadr = obj.quadr_Kp * (A(2, 3) * C(3, 1) + A(2, 4) * C(4, 1) + B(2, 1));
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

