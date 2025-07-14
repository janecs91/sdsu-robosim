classdef Walk < Transformation
    properties
        mode = 'walk';
    end
    properties(Constant)
        %modeKeys = {'ankle_1', 'ankle_2', 'ankle_3', 'ankle_4'}
        %modeValues = {0 0 0 0}
        %modeConstants = containers.Map(Walk.modeKeys, Walk.modeValues)
    end
    methods
        function obj = Walk(transformationDirectory)
            dhTable = Walk.get_dh();
            obj@Transformation(transformationDirectory, dhTable);
            obj = obj.save();
        end
    end
    methods(Static)
        function dhTable = get_dh()
            % symbols
            syms a_0 a_1 a_2 a_3 a_4 d_0 d_1 d_5;
            % dhTable
            dhTable = cell(4,1);
            % theta d alpha r
            theta2 = [deg2rad(-45) deg2rad(45) deg2rad(135) deg2rad(-135)];
            alpha1 = [deg2rad(90) deg2rad(-90) deg2rad(-90) deg2rad(90)];
            alpha2 = [deg2rad(-90) deg2rad(90) deg2rad(90) deg2rad(-90)];
            %theta2 = [-45 45 135 -135];
            %alpha1 = [90 -90 -90 90];
            %alpha2 = [-90 90 90 -90];
            r1 = [a_0 a_0 -a_0 -a_0];
            for leg = 1:4
                dhTable(leg) = {[0 d_0 alpha1(leg) r1(leg); ...
                    sym(sprintf('waist_%d', leg))+theta2(leg) -d_1 alpha2(leg) 0; ...
                    deg2rad(90)-sym(sprintf('hip_%d', leg)) 0 deg2rad(90) a_2; ...
                    sym(sprintf('knee_%d', leg))-deg2rad(180) 0 0 a_3; ...
                    0 0 0 a_3]};
            end
        end
    end
end