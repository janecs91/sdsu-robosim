classdef Pull < Transformation
    properties
        mode = 'pull';
    end
    properties(Constant)
        %modeKeys = {'ankle_1', 'ankle_2', 'ankle_3', 'ankle_4'}
        %modeValues = {0 0 0 0}
        %modeConstants = containers.Map(Walk.modeKeys, Walk.modeValues)
    end
    methods
        function obj = Pull(transformationDirectory)
            dhTable = Pull.get_dh();
            obj@Transformation(transformationDirectory, dhTable);
            obj = obj.save();
        end
    end
    methods(Static)
        function dhTable = get_dh()
            dhTable = cell(4,1);
            walkDH = Walk.get_dh();
            rollDH = Roll.get_dh();
            for leg=1:2
                dhTable(leg) = walkDH(leg);
            end
            for leg=3:4
                dhTable(leg) = rollDH(leg);
            end
        end
    end
end