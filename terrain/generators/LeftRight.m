classdef LeftRight < TerrainGenerator
    properties(Constant)
        % vars: width, height, amplitude, frequency, path
        nameLeftRight = 'leftright_%d_%d_%d_%s';
        argNames = {'amplitude', 'path'};
    end
    methods
        function obj = LeftRight(directory)
            obj@TerrainGenerator(directory);
        end
        function fileAddress = generate(obj, xRange, yRange, scale, amplitude, frequency, path)
            initX = xRange(1);
            initY = yRange(1);
            scaleX = scale;
            scaleY = scale;
            maxX = xRange(2);
            maxY = yRange(2);
            lengthX = length(initX:scaleX:maxX);
            lengthY = length(initY:scaleY:maxY);
            
            elevationMatrix = zeros(lengthX, lengthY);
            [pathWidth, pathHeight, pathMaxY, pathMinY] = path.get_size();
            pathCount = path.get_path_points_count();
            startPoint = path.get_first_point();
            endPoint = path.get_last_point();
            pathMinYScaled = ceil(startPoint(2)/scaleY);
            pathMaxYScaled = ceil(endPoint(2)/scaleY);
            pathWidthByIndex = ceil(startPoint(1)/scaleX):ceil(endPoint(1)/scaleX);
            pathHeightByIndex = pathMinYScaled:pathMaxYScaled;
            
            elevationMatrix(:,:) = amplitude;
            lastXIndex = 1;
            for pathIndex = 1:pathCount
                %z = amplitude.*sin(frequency.*x);
                pathPoint = path.get_point(pathIndex);
                pathPointX = ceil(pathPoint(1)/scaleX);
                pathPointY = ceil(pathPoint(2)/scaleY);
                xIndices = lastXIndex:pathPointX;
                lastXIndex = pathPointX;
                yIndices = 1:pathPointY;
                %yIndices = pathPointY:lengthY;
                lengthXIndices = size(xIndices,2);
                lengthYIndices = size(yIndices,2);
                elevationMatrix(xIndices,yIndices) = ones(lengthXIndices,lengthYIndices)*-amplitude;
            end
            xIndices = lastXIndex:lengthX;
            lengthXIndices = size(xIndices,2);
            elevationMatrix(xIndices,yIndices) = ones(lengthXIndices,lengthYIndices)*-amplitude;
            
            slipMatrix = zeros(size(elevationMatrix));
            
            % save
            terrainName = sprintf(obj.name, maxX, maxY, amplitude, path.get_name());
            fileAddress = sprintf(obj.saveFileAddress,terrainName);
            save(fileAddress, 'initX', 'initY', 'maxX', 'maxY', 'scaleX', 'scaleY', ...
            'elevationMatrix', 'slipMatrix');
        end
    end
end