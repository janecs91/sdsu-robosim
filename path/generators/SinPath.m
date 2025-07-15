classdef SinPath < PathGenerator
    properties(Constant)
        % vars: startX, startY, endX, endY, amplitude
        name = 'sin_%d_%d_%d_%d_%.0f';
        argNames = {'amplitude'};
        isSameStartYEndY = true;
    end
    methods
        function obj = SinPath(directory)
            obj@PathGenerator(directory);
        end
        function fileAddress = generate(obj, startPoint, endPoint, stepSize, amplitude)
            startX = startPoint(1);
            endX = endPoint(1);
            startY = (amplitude/2)+obj.robotSize;
            endY = startY;
            
            phaseShiftX = startX+(pi/2);
            phaseShiftY = amplitude+startY;
            period = (endX-startX)*(3/pi);
                     
            %pathLength = ceil(pi*period);
            %pathX = linspace(startX, endX, pathLength);
            stepSize = 1;
            pathX = startX:stepSize:endX;
            pathLength = size(pathX,2);
            pathPoints = zeros(pathLength,2);
            for i=1:pathLength
                x = pathX(i);
                pathY = amplitude*sin(((2*pi/period)*x)+phaseShiftX);
                pathY = pathY+phaseShiftY;
                pathPoints(i,:) = [pathX(i) pathY];
            end
            pathName = sprintf(obj.name, startX, startY, endX, endY, ...
                amplitude);
            fileAddress = sprintf(obj.saveFileAddress, pathName);
            save(fileAddress, 'pathPoints');
        end
    end
end