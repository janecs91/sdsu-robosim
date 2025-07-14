classdef StepAdjuster
properties
    % limit ranges
    safetyValue = 0.9;
    maxLegLength = 48;
end
methods
    function obj = StepAdjuster(safetyValue)
        addpath('robot/legged/step_strategies');
        addpath('robot/kinematics');
        obj.safetyValue = safetyValue;
    end
end
end
