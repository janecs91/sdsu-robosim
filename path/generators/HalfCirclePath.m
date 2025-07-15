classdef HalfCirclePath < PathGenerator
    properties(Constant)
        % vars: startX, startY, endX, endY, sign
        name = 'halfcirc_%d_%d_%d_%d_%d';
        argNames = {};
    end
    methods
        function obj = HalfCirclePath(directory)
            obj@PathGenerator(directory);
        end
        function fileAddress = generate(obj, startPoint, endPoint, stepSize, sign)
            startX = startPoint(1);
            startY = startPoint(2);
            endX = endPoint(1);
            endY = startY;
            
            diameter = abs(endX - startX);
            radius = diameter/2;
            radius_squared = radius.^2;
            
            pathGenName = obj.name;
            if sign < 0
                pathGenName = strcat(obj.name,'2');
                startY = radius+startY;
            end
            
            pathLength = ceil(pi*radius);
            pathX = linspace(startX, endX, pathLength);
            pathPoints = zeros(pathLength,2);
            for i=1:pathLength
                x = pathX(i)-startX;
                pathY = sqrt(radius_squared - (x-radius)^2);
                pathY = sign*pathY;
                pathY = pathY + startY;
                pathPoints(i,:) = [pathX(i) pathY];
            end
            pathName = sprintf(pathGenName, startX, startY, endX, endY);
            fileAddress = sprintf(obj.saveFileAddress, pathName);
            save(fileAddress, 'pathPoints');
        end
    end
end