classdef Flat < TerrainGenerator
    properties(Constant)
        % vars: width, height
        name = 'flat_%d_%d';
        argNames = {};
    end
    % fix get terrain name (consolidate names)
    properties
    end
    methods
        function obj = Flat(directory)
            obj@TerrainGenerator(directory);
        end
        function name = get_name(obj, fromPath, path, type, varargin)
            name = sprintf(Flat.name, width, height, args{:});
        end
        function fileAddress = generate(obj, xRange, yRange, scale)
            initX = xRange(1);
            initY = yRange(1);
            scaleX = scale;
            scaleY = scale;
            maxX = xRange(2);
            maxY = yRange(2);
            lengthX = length(initX:scaleX:maxX);
            lengthY = length(initY:scaleY:maxY);
            elevationMatrix = zeros(lengthX, lengthY);
            slipMatrix = zeros(size(elevationMatrix));
            terrainName = sprintf(obj.name, maxX, maxY);
            fileAddress = sprintf(obj.saveFileAddress,terrainName);
            save(fileAddress, 'initX', 'initY', 'maxX', 'maxY', 'scaleX', 'scaleY', ...
            'elevationMatrix', 'slipMatrix');
        end
    end
end