%% Symbolic inverse matrices -- NOT IN USE
classdef InvABy < SaveTransformation
    properties
    end
    properties
        %% Save File Properties
        savePrefix = 'invaby_';
    end
    methods
        function obj = InvABy(obj@InvABy(transformationDirectory););
            obj@SaveTransformation(obj@InvABy(transformationDirectory););
        end
    end
end