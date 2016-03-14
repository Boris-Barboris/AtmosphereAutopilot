classdef ang_vel_pitch_yaw < ang_vel_controller
    
    properties (SetAccess = public)
        max_v_construction = 0.5;
        max_aoa = 15.0;
        max_g = 15.0;
        moderate_aoa = true;
        moderate_g = true;
        
        stable = true;
        res_max_aoa = 0.1;
        res_min_aoa = -0.1;
        res_equilibr_v_upper = 0.0;
        res_equilibr_v_lower = 0.0;
        
        max_input_aoa = 0.1;
        min_input_aoa = -0.1;
        max_input_v = 0.0;
        min_input_v = 0.0;
        
        max_g_aoa = 0.1;
        min_g_aoa = -0.1;
        max_g_v = 0.0;
        min_g_v = 0.0;
        
        max_aoa_v = 0.0;
        min_aoa_v = 0.0;
    end
    
    methods (Access = public)
        function c = ang_vel_pitch_yaw(ax, ac)
            c@ang_vel_controller(ac);
            c.axis = ax;
        end
        
        function cntrl = eval(obj, target, dt)
            obj.target_vel = target;
            obj.update_pars();
            if (obj.user_input)
                target = moderate(target);
            end
            obj.output_acc = obj.get_desired_acc(target, dt);
            cntrl = obj.acc_c.eval(obj.output_acc, dt);
        end
    end
    
    methods (Access = private)
        function update_pars(obj)
            rad_max_aoa = obj.max_aoa * pi / 180.0;
            obj.res_max_aoa = 100.0;
            obj.res_min_aoa = -100.0;
            obj.res_equilibr_v_upper = 0.0;
            obj.res_equilibr_v_lower = 0.0;
            
            cur_aoa = obj.model.aoa(axis + 1);
            abs_cur_aoa = abs(cur_aoa);
            moderated = false;
            
            if (obj.axis == 0)
                A = obj.model.pitch_A;
                B = obj.model.pitch_B;
                C = obj.model.pitch_C;
            else
                A = obj.model.yaw_A;
                B = obj.model.yaw_B;
                C = obj.model.yaw_C;
            end
            
            % evaluate equilibrium regimes and moderation limits
            if (obj.moderate_aoa && obj.model.dyn_pressure > 100.0)
                moderated = true;
                if (abs_cur_aoa < rad_max_aoa * 1.5)
                    % max input section
                    % get equilibrium aoa and angular_v for 1.0 input
                    eq_A = zeros(2);
                    eq_A(1, 1) = A(1, 1);
                    eq_A(1, 2) = A(1, 2);
                    eq_A(2, 1) = A(1, 0);
                    eq_A(2, 2) = 0.0;
                    eq_B = [-(A(1, 3) + A(1, 4) + B(1, 1) + C(1, 1)); -(A(2, 3) + A(2, 4) + B(2, 1) + C(2, 1))];
                    eq_x = eq_A \ eq_B;
                    if (eq_x(1, 1) < 0.0)
                        obj.stable = false;
                        obj.min_input_aoa = 0.6 * eq_x(1, 1);
                        obj.min_input_v = 0.6 * eq_x(2, 1);
                    else
                        obj.stable = true;
                        obj.max_input_aoa = 0.98 * eq_x(1, 1);
                        obj.max_input_v = 0.98 * eq_x(2, 1);
                    end                    
                    % get equilibrium aoa and angular_v for -1.0 input
                    eq_B = [A(1, 3) + A(1, 4) + B(1, 1) - C(1, 1); A(2, 3) + A(2, 4) + B(2, 1) - C(2, 1)];
                    eq_x = eq_A \ eq_B;
                    if (eq_x(1, 1) >= 0.0)
                        obj.max_input_aoa = 0.6 * eq_x(1, 1);
                        obj.max_input_v = 0.6 * eq_x(2, 1);
                    else
                        obj.min_input_aoa = 0.98 * eq_x(1, 1);
                        obj.min_input_v = 0.98 * eq_x(2, 1);
                    end
                    
                    % max aoa section
                    eq_A = [A(1, 2), A(1, 3) + A(1, 4) + B(1, 1); A(2, 2), A(2, 3) + A(2, 4) + B(2, 1)];
                    eq_B = [-(A(1, 1) * rad_max_aoa + C(1, 1)); -(A(2, 1) * rad_max_aoa + C(2, 1))];
                    eq_X = eq_A \ eq_B;
                    obj.max_aoa_v = eq_X(1, 1);
                    eq_B = [A(1, 1) * rad_max_aoa - C(1, 1); A(2, 1) * rad_max_aoa - C(2, 1)];
                    eq_X = eq_A \ eq_B;
                    obj.min_aoa_v = eq_X(1, 1);
                end
                
                % let's apply moderation with controllability region
                if (obj.max_input_aoa < obj.res_max_aoa)
                    obj.res_max_aoa = obj.max_input_aoa;
                    obj.res_equilibr_v_upper = obj.max_input_v;
                end
                if (obj.min_input_aoa > obj.res_min_aoa)
                    obj.res_min_aoa = obj.min_input_aoa;
                    obj.res_equilibr_v_lower = obj.min_input_v;
                end
                
                % let's apply simple AoA moderation
                if (rad_max_aoa < obj.res_max_aoa)
                    obj.res_max_aoa = rad_max_aoa;
                    obj.res_equilibr_v_upper = obj.max_aoa_v;
                end
                if (-rad_max_aoa > obj.res_min_aoa)
                    obj.res_min_aoa = -rad_max_aoa;
                    obj.res_equilibr_v_lower = obj.min_aoa_v;
                end
            end
            
            if (obj.moderate_g && obj.model.dyn_pressure > 100.0)
                moderated = true;
                if (abs(A(1, 1)) > 1e-5 && abs_cur_aoa < rad_max_aoa * 1.5)
                    if (obj.axis == 0)
                        gravity_acc = obj.model.pitch_gravity_acc;
                    else
                        gravity_acc = obj.model.yaw_gravity_acc;
                    end
                    % get equilibrium aoa and angular v for max_g g-force
                    obj.max_g_v = (obj.max_g_force * 9.81 + gravity_acc) / obj.model.velocity_magn;
                    obj.min_g_v = (-obj.max_g_force * 9.81 + gravity_acc) / obj.model.velocity_magn;
                    eq_A = [A(1, 1), A(1, 3) + A(1, 4) + B(1, 1); A(2, 1), A(2, 3) + A(2, 4) + B(2, 1)];
                    eq_B = [-(obj.max_g_v + C(1, 1)); -C(2, 1)];
                    eq_X = eq_A \ eq_B;
                    obj.max_g_aoa = eq_X(1, 1);
                    eq_B = [-(obj.min_g_v + C(1, 1)); -C(2, 1)];
                    eq_X = eq_A \ eq_B;
                    obj.min_g_aoa = eq_X(1, 1);
                end
                
                % apply g-force moderation parameters
                if (obj.max_g_aoa < 2.0 && obj.max_g_aoa > 0.0 && obj.min_g_aoa > -2.0 && obj.max_g_aoa > obj.min_g_aoa)
                    if (obj.max_g_aoa < obj.res_max_aoa)
                        obj.res_max_aoa = obj.max_g_aoa;
                        obj.res_equilibr_v_upper = obj.max_g_v;
                    end
                    if (obj.min_g_aoa > obj.res_min_aoa)
                        obj.res_min_aoa = obj.min_g_aoa;
                        obj.res_equilibr_v_lower = obj.min_g_v;
                    end
                end
            end
            
            % transit velocity evaluation
            
        end
        
        function desired_v = moderate(obj, des_v)
            
        end
        
        function des_acc = get_desired_acc(obj, des_v, dt)
                
        end
    end
    
end

