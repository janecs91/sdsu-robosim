classdef Sin < TerrainGenerator
    properties(Constant)
        % vars: width, height, amplitude, frequency
        name = 'sin_%d_%d_%d_%.3f';
        argNames = {'amplitude', 'frequency'};
    end
    methods
        function obj = Sin(directory)
            obj@TerrainGenerator(directory);
        end
        function fileAddress = generate(obj, xRange, yRange, scale, amplitude, frequency)
            %period = (2*pi)*(1/frequency);
            initX = xRange(1);
            initY = yRange(1);
            scaleX = scale;
            scaleY = scale;
            maxX = xRange(2);
            maxY = yRange(2);
            lengthX = length(initX:scaleX:maxX);
            lengthY = length(initY:scaleY:maxY);
            
            elevationMatrix = zeros(lengthX, lengthY);
            xIndex = 1;
            for x = initX:scaleX:maxX
                z = amplitude.*sin(frequency.*x);
                elevationsAcrossY = ones(1,lengthY)*z;
                elevationMatrix(xIndex,:) = elevationsAcrossY;
                xIndex = xIndex + 1;
            end
            
            slipMatrix = zeros(size(elevationMatrix));
            
            % save
            terrainName = sprintf(obj.name, maxX, maxY, amplitude, frequency);
            fileAddress = sprintf(obj.saveFileAddress,terrainName);
            save(fileAddress, 'initX', 'initY', 'maxX', 'maxY', 'scaleX', 'scaleY', ...
            'elevationMatrix', 'slipMatrix');
        end
    end
end