classdef Ramp < TerrainGenerator
    properties(Constant)
        % vars: width, height, amplitude
        name = 'ramp_%d_%d_%d';
        argNames = {'amplitude'};
    end
    methods
        function obj = Ramp(directory)
            obj@TerrainGenerator(directory);
        end
        function fileAddress = generate(obj, xRange, yRange, scale, amplitude)
            initX = xRange(1);
            initY = yRange(1);
            scaleX = scale;
            scaleY = scale;
            maxX = xRange(2);
            maxY = yRange(2);
            lengthX = length(initX:scaleX:maxX);
            lengthY = length(initY:scaleY:maxY);
            xElevationRow = linspace(0,amplitude,lengthX);
            %elevationMatrix = zeros(length(initX:scaleX:maxX), length(initY:scaleY:maxY));
            elevationMatrix = repmat(xElevationRow,lengthY,1);
            elevationMatrix = elevationMatrix';
            slipMatrix = zeros(size(elevationMatrix));
            terrainName = sprintf(obj.name, maxX, maxY);
            fileAddress = sprintf(obj.saveFileAddress,terrainName);
            save(fileAddress, 'initX', 'initY', 'maxX', 'maxY', 'scaleX', 'scaleY', ...
            'elevationMatrix', 'slipMatrix');
        end
    end
end