classdef ang_vel_controller < handle
    
    properties (SetAccess = public)
        model;                      % aircraft_model instance
        acc_c;                      % corresponding angular acceleration controller
        axis = 0;                   % 0 - pitch 1 - roll 2 - yaw
        target_vel = 0.0;
        output_acc = 0.0;
        user_controlled = false;    % true when target is user control
    end
    
    methods (Access = public)
        function c = ang_vel_controller(ac)
            c.acc_c = ac;
            c.model = ac.model;
        end
    end
    
end

