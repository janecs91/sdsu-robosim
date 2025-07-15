classdef Roll < Transformation
    properties
        mode = 'roll';
    end
    methods
        function obj = Roll(transformationDirectory)
            dhTable = Roll.get_dh();
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
            r1 = [a_0 a_0 -a_0 -a_0];
            for leg = 1:4
                dhTable(leg) = {[0 d_0 alpha1(leg) r1(leg); ...
                    theta2(leg) -d_1 alpha2(leg) 0; ...
                    deg2rad(90)-sym(sprintf('hip_%d', leg)) 0 deg2rad(90) a_2; ...
                    2*sym(sprintf('hip_%d', leg)) 0 0 a_3; ...
                    deg2rad(270)-sym(sprintf('hip_%d', leg)) 0 0 a_4; ...
                    sym(sprintf('steering_%d', leg)) -d_5 deg2rad(-90) 0]};
            end
        end
    end
end