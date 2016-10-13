classdef ang_vel_pitch_yaw < ang_vel_controller
    
    properties (SetAccess = public)
        max_v_construction = 0.7;
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
        
        transit_max_v = 0.7;
        transit_v_mult = 0.5;
        
        % get_desired_acc section
        quadr_Kp = 0.45;
        relaxation_k = -1.0;
        relaxation_Kp = 0.5;
        relax_count = 0;
        
        kacc_quadr = 0.0;
        
        already_preupdated = false;
    end
    
    methods (Access = public)
        function c = ang_vel_pitch_yaw(ax, ac)
            c@ang_vel_controller(ac);
            c.axis = ax;
        end
        
        function cntrl = eval(obj, target, target_deriv, dt)
            obj.target_vel = target;
            obj.update_pars();
            if (obj.user_controlled)
                target = moderate(target);
            end
            acc_unconst = obj.get_desired_acc(target, target_deriv, dt);
            acc_constrained = obj.get_desired_acc(sign(target) * obj.max_v_construction, 0.0, dt);
            if (sign(target) * acc_constrained < sign(target) * acc_unconst)
                obj.output_acc = acc_constrained;
            else
                obj.output_acc = acc_unconst;
            end            
            cntrl = obj.acc_c.eval(obj.output_acc, dt);
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
            
            rad_max_aoa = obj.max_aoa * pi / 180.0;
            obj.res_max_aoa = 100.0;
            obj.res_min_aoa = -100.0;
            obj.res_equilibr_v_upper = 0.0;
            obj.res_equilibr_v_lower = 0.0;
            
            cur_aoa = obj.model.aoa(obj.axis + 1);
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
            
            % AoA section
            if (obj.moderate_aoa && obj.model.dyn_pressure > 100.0)
                moderated = true;
                if (abs_cur_aoa < rad_max_aoa * 1.5)
                    % max input section
                    % get equilibrium aoa and angular_v for 1.0 input
                    eq_A = zeros(2);
                    eq_A(1, 1) = A(1, 1);
                    eq_A(1, 2) = A(1, 2);
                    eq_A(2, 1) = A(2, 1);
                    eq_A(2, 2) = 0.0;
                    eq_B = [-(A(1, 3) + A(1, 4) + B(1, 1) + C(1, 1)); -(A(2, 3) + A(2, 4) + B(2, 1) + C(2, 1))];
                    eq_x = eq_A \ eq_B;
                    if (eq_x(1, 1) < 0.0)
                        obj.stable = false;
                        obj.min_input_aoa = 0.6 * eq_x(1, 1);
                        obj.min_input_v = 0.6 * eq_x(2, 1);
                    else
                        obj.stable = true;
                        obj.max_input_aoa = eq_x(1, 1);
                        obj.max_input_v = eq_x(2, 1);
                    end                    
                    % get equilibrium aoa and angular_v for -1.0 input
                    eq_B = [A(1, 3) + A(1, 4) + B(1, 1) - C(1, 1); A(2, 3) + A(2, 4) + B(2, 1) - C(2, 1)];
                    eq_x = eq_A \ eq_B;
                    if (eq_x(1, 1) > 0.0)
                        obj.max_input_aoa = 0.6 * eq_x(1, 1);
                        obj.max_input_v = 0.6 * eq_x(2, 1);
                    else
                        obj.min_input_aoa = eq_x(1, 1);
                        obj.min_input_v = eq_x(2, 1);
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
                
                % apply simple AoA moderation
                if (rad_max_aoa < obj.res_max_aoa)
                    obj.res_max_aoa = rad_max_aoa;
                    obj.res_equilibr_v_upper = obj.max_aoa_v;
                end
                if (-rad_max_aoa > obj.res_min_aoa)
                    obj.res_min_aoa = -rad_max_aoa;
                    obj.res_equilibr_v_lower = obj.min_aoa_v;
                end
            end
            
            
            % G force section
            if (obj.moderate_g && obj.model.dyn_pressure > 100.0)
                moderated = true;
                if ((abs(A(1, 1)) > 1e-5) && (abs_cur_aoa < rad_max_aoa * 1.5))
                    if (obj.axis == 0)
                        gravity_acc = obj.model.pitch_gravity_acc;
                    else
                        gravity_acc = obj.model.yaw_gravity_acc;
                    end
                    % get equilibrium aoa and angular v for max_g g-force
                    obj.max_g_v = (obj.max_g * 9.81 + gravity_acc) / obj.model.velocity_magn;
                    obj.min_g_v = (-obj.max_g * 9.81 + gravity_acc) / obj.model.velocity_magn;
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
            if ((abs_cur_aoa < rad_max_aoa * 1.5) && moderated)
                transit_max_aoa = min(rad_max_aoa, obj.res_max_aoa);
                state_mat = zeros(4, 1);
                if (obj.stable)
                    state_mat(1, 1) = transit_max_aoa / 3.0;
                else
                    state_mat(1, 1) = transit_max_aoa;
                end
                state_mat(3, 1) = -1.0;
                state_mat(4, 1) = -1.0;
                state_mat(1, 1) = -1.0;
                input_mat = -1.0;
                stderiv = A * state_mat + B * input_mat + C;
                acc = stderiv(2);
                dyn_max_v = obj.transit_v_mult * sqrt(2.0 * transit_max_aoa * (-acc));
                if (isnan(dyn_max_v))
                    obj.transit_max_v = obj.max_v_construction;
                else
                    % for cases when static authority is too small to comply to long-term dynamics, 
                    % we need to artificially increase it
                    if (dyn_max_v < obj.res_equilibr_v_upper * 1.2 || ...
                        dyn_max_v < -obj.res_equilibr_v_lower * 1.2)
                        dyn_max_v = 1.2 * max(abs(obj.res_equilibr_v_upper), abs(obj.res_equilibr_v_lower));
                    end
                    dyn_max_v = aircraft_model.clamp(dyn_max_v, -obj.max_v_construction, obj.max_v_construction);
                    obj.transit_max_v = dyn_max_v;
                end
            else
                obj.transit_max_v = obj.max_v_construction;    
            end
            
            if (obj.model.aero_model) %if FAR
                kacc = obj.quadr_Kp * (A(2, 3) * B(3, 1) + A(2, 4) * C(4, 1) + B(2, 1));
            else
                kacc = obj.quadr_Kp * (A(2, 3) * C(3, 1) + A(2, 4) * C(4, 1) + B(2, 1));
            end
            obj.kacc_quadr = abs(kacc);
        end
        
        function desired_v = moderate(obj, des_v)
            % not implemented
            desired_v = des_v;
        end
        
        function des_acc = get_desired_acc(obj, des_v, target_deriv, dt)
            cur_v = obj.model.angular_vel(obj.axis + 1);
            v_error = cur_v - des_v;
            k = sign(v_error) * obj.kacc_quadr;
            if (k == 0.0)
                des_acc = target_deriv;
                return;
            end
            d = target_deriv;
            b = d / 2.0 / k;
            s = (-d + sign(k) * 2.0 * sqrt(k * v_error)) / 2.0 / k;
            c = v_error - k * s^2;
            quadr_x = 0.0;
            if (b + s <= dt)
                desired_deriv = (d * dt - k * (quadr_x - s) ^ 2 - c) / dt;
            else
                intersect_x = dt;
                desired_deriv = k * ((intersect_x - s) ^ 2 - (quadr_x - s) ^ 2) / dt;
            end
            des_acc = desired_deriv;
        end
    end
    
end

