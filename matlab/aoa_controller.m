classdef aoa_controller < handle
    
    properties (SetAccess = public)
        model;                      % aircraft_model instance
        vel_c;                      % corresponding angular velocity controller
        axis = 0;                   % 0 - pitch 1 - roll 2 - yaw
        output_vel = 0.0;
        output_acc = 0.0;
        user_controlled = false;    % true when target is user control
        
        cur_aoa_equilibr = 0.0;     % equilibrium angular velocity to stay on current AoA
        
        
        already_preupdated = false;
    end
    
    methods (Access = public)
        function c = aoa_controller(vc, axis)
            c.vel_c = vc;
            c.model = vc.model;
            c.axis = axis;
        end
        
        function cntrl = eval(obj, target_aoa, target_vel, dt)
            obj.target_aoa = target_aoa;
            obj.vel_c.preupdate();
            obj.update_pars();
            obj.output_vel, obj.output_acc = obj.get_desired_output(target_aoa, dt);
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
            
        end
        
        function out_vel, out_acc = get_desired_output(obj, target_aoa, dt)
            
        end
        
    end
    
end

