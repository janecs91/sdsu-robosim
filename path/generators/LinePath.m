classdef LinePath < PathGenerator
    properties(Constant)
        % vars: startX, startY, endX, endY
        name = 'line_%d_%d_%d_%d';
        argNames = {};
    end
    methods
        function obj = LinePath(directory)
            obj@PathGenerator(directory);
        end
        function fileAddress = generate(obj, startPoint, endPoint, stepSize)
            startX = startPoint(1);
            startY = startPoint(2);
            endX = endPoint(1);
            endY = endPoint(2);
            pathX = startX:stepSize:endX;
            pathLength = size(pathX,2);
            pathY = linspace(startY, endY, pathLength);
            pathPoints = zeros(pathLength,2);
            for i=1:pathLength
                pathPoints(i,:) = [pathX(i) pathY(i)];
            end
            pathName = sprintf(obj.name, startX, startY, endX, endY);
            fileAddress = sprintf(obj.saveFileAddress, pathName);
            save(fileAddress, 'pathPoints');
        end
    end
end