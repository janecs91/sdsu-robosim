classdef LeftRightSin < TerrainGenerator
    properties(Constant)
        % vars: width, height, amplitude, frequency, path
        nameLeftRight = 'leftrightsin_%d_%d_%d_%s';
        argNames = {'amplitude', 'path'};
    end
    methods
        function obj = LeftRightSin(directory)
            obj@TerrainGenerator(directory);
        end
        function fileAddress = generate(obj, xRange, yRange, scale, amplitude, frequency, path)
            %period = (2*pi)*(1/frequency);
            initX = xRange(1);
            initY = yRange(1);
            scaleX = scale;
            scaleY = scale;
            maxX = xRange(2);
            maxY = yRange(2);
            lengthX = length(initX:scaleX:maxX);
            lengthY = length(initY:scaleY:maxY);
            
            %% set half to sin
            elevationMatrix = zeros(lengthX, lengthY);
            xIndex = 1;
            for x = initX:scaleX:maxX
                z = amplitude.*sin(frequency.*x);
                elevationsAcrossY = ones(1,lengthY)*z;
                elevationMatrix(xIndex,:) = elevationsAcrossY;
                xIndex = xIndex + 1;
            end
            
            %% set other half to 0
            [pathWidth, pathHeight, pathMaxY, pathMinY] = path.get_size();
            pathCount = path.get_path_points_count();
            startPoint = path.get_first_point();
            endPoint = path.get_last_point();
            pathMinYScaled = ceil(startPoint(2)/scaleY);
            pathMaxYScaled = ceil(endPoint(2)/scaleY);
            pathWidthByIndex = ceil(startPoint(1)/scaleX):ceil(endPoint(1)/scaleX);
            pathHeightByIndex = pathMinYScaled:pathMaxYScaled;
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
                elevationMatrix(xIndices,yIndices) = ones(lengthXIndices,lengthYIndices)*0;
            end
            xIndices = lastXIndex:lengthX;
            lengthXIndices = size(xIndices,2);
            elevationMatrix(xIndices,yIndices) = ones(lengthXIndices,lengthYIndices)*0;
            
            slipMatrix = zeros(size(elevationMatrix));
            
            % save
            terrainName = sprintf(obj.name, maxX, maxY, amplitude, frequency);
            fileAddress = sprintf(obj.saveFileAddress,terrainName);
            save(fileAddress, 'initX', 'initY', 'maxX', 'maxY', 'scaleX', 'scaleY', ...
            'elevationMatrix', 'slipMatrix');
        end
    end
end