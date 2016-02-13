classdef ang_acc_controller < handle
    %UNTITLED Base class for angular acceleration controllers
    %   Detailed explanation goes here
    
    properties (SetAccess = public)
        model;                      % aircraft_model instance
        axis = 0;                   % 0 - pitch 1 - roll 2 - yaw
        target_acc = 0.0;
        predicted_acc = 0.0;
        output_control = 0.0;
    end
    
    methods (Access = public)
        function c = ang_acc_controller(m)
            c.model = m;
        end
    end    
end

