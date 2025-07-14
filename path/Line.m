classdef Line
    % not used by path generator or path classes
    % this is a helper utility class used by step adjuster class for walking robot
    properties
    end
    methods(Static)
        function [a, b] = get_parametric_equation(angle)
            a = cos(angle);
            b = sin(angle);
        end
        function [plusPoint, minusPoint] = get_parallel_point_from_distance(x, y, angle, distance)
            orthogonalAngle = angle+deg2rad(90);
            [a, b] = Line.get_parametric_equation(orthogonalAngle);
            xPlus = x+a*distance;
            yPlus = y+b*distance;
            xMinus = x+a*(-distance);
            yMinus = y+b*(-distance);
            plusPoint = [xPlus yPlus];
            minusPoint = [xMinus yMinus];
        end
        function [vector, magnitude] = get_vector(x1, y1, x2, y2)
            vector = [x2-x1 y2-y1];
            magnitude = sqrt(vector(1)^2+vector(2)^2);
        end
        function unitVector = get_unit_vector(x1, y1, x2, y2)
            [vector, magnitude] = Line.get_vector(x1, y1, x2, y2);
            unitVector = vector/magnitude;
            %fprintf("vector: %.2f %.2f, magnitude: %.2f, unit vector: %.2f %.2f\n", vector, magnitude, unitVector);
        end
    end
end