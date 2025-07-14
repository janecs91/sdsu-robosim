classdef TerrainGenerator
    properties(Constant)
        savePath = 'input_terrain/terrain_%s.mat';
        % names
        delimiter = '_';
        % vars: width, height
        nameFlat = 'flat_%d_%d';
        % vars: width, height, amplitude
        nameRamp = 'ramp_%d_%d_%d';
        % vars: width, height, amplitude, frequency
        nameSin = 'sin_%d_%d_%d_%.3f';
        % vars: width, height, filter size, elevation range
        nameRandom = 'random_%d_%d_%d_%d';
        % vars: width, height, amplitude, frequency, path
        nameLeftRight = 'leftright_%d_%d_%d_%s';
        % vars: width, height, amplitude, frequency, path
        nameHalfSin = 'halfsin_%d_%d_%d_%s';
        % args
        argNames = {'width', 'height', 'cellsize', 'amplitude', 'frequency', ...
                'randFilterSize', 'elevationChangeRange'};
        % options
        terrainPadding = [100 100];
    end
    % fix get terrain name (consolidate names)
    properties
        saveFileAddress;
        directory = 'input_terrain';
        prefix = 'terrain';
    end
    methods
        function obj = TerrainGenerator(directory)
            obj.saveFileAddress = string(directory) + "/" + string(obj.prefix) + "_%s.mat";
        end
        function saveFileAddress = get_save_address(obj)
            saveFileAddress = obj.saveFileAddress;
        end
        %% Helper Generator Functions
        function fileAddress = gen_from_string(obj, input)
            [terrainType, xRange, yRange, extraArgs] = TerrainGenerator.get_data_from_string(input);
            scale = 10;
            fileAddress = obj.gen_from_options(terrainType, xRange, yRange, scale, extraArgs);
        end
        function fileAddress = gen_from_options(obj, genFromPath, path, terrainType, width, height, ...
                scale, varargin)
            if genFromPath == true
                [width, height] = TerrainGenerator.get_terrain_size_from_path(path);
            end
            xRange = [0 width];
            yRange = [0 height];
            if strcmp(terrainType, 'flat')
                fileAddress = obj.gen_flat(xRange, yRange, scale);
            elseif strcmp(terrainType, 'ramp')
                amplitude = varargin{1};
                fileAddress = obj.gen_ramp(xRange, yRange, scale, amplitude);
            elseif strcmp(terrainType, 'sin')
                amplitude = varargin{1};
                frequency = varargin{2};
                fileAddress = obj.gen_sin(xRange, yRange, scale, amplitude, frequency);
            elseif strcmp(terrainType, 'random')
                filterSize = varargin{1};
                elevationChangeRange = varargin{2};
                fileAddress = obj.gen_random(xRange, yRange, scale, filterSize, elevationChangeRange);
            elseif strcmp(terrainType, 'leftright')
                amplitude = varargin{1};
                frequency = varargin{2};
                fileAddress = obj.gen_different_left_right(xRange, yRange, scale, amplitude, frequency, path);
            elseif strcmp(terrainType, 'halfsin')
                amplitude = varargin{1};
                frequency = varargin{2};
                fileAddress = obj.gen_halfsin(xRange, yRange, scale, amplitude, frequency, path);
            end
        end
        %% Generator Functions
        function fileAddress = gen_flat(obj, xRange, yRange, scale)
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
            terrainName = sprintf(TerrainGenerator.nameFlat, maxX, maxY);
            fileAddress = sprintf(obj.saveFileAddress,terrainName);
            save(fileAddress, 'initX', 'initY', 'maxX', 'maxY', 'scaleX', 'scaleY', ...
            'elevationMatrix', 'slipMatrix');
        end
        function fileAddress = gen_ramp(obj, xRange, yRange, scale, amplitude)
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
            terrainName = sprintf(TerrainGenerator.nameRamp, maxX, maxY);
            fileAddress = sprintf(obj.saveFileAddress,terrainName);
            save(fileAddress, 'initX', 'initY', 'maxX', 'maxY', 'scaleX', 'scaleY', ...
            'elevationMatrix', 'slipMatrix');
        end
        function fileAddress = gen_sin(obj, xRange, yRange, scale, amplitude, frequency)
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
            terrainName = sprintf(TerrainGenerator.nameSin, maxX, maxY, amplitude, frequency);
            fileAddress = sprintf(obj.saveFileAddress,terrainName);
            save(fileAddress, 'initX', 'initY', 'maxX', 'maxY', 'scaleX', 'scaleY', ...
            'elevationMatrix', 'slipMatrix');
        end
        function fileAddress = gen_random(obj, xRange, yRange, scale, filterSize, elevationChangeRange)
            if nargin < 4
                filterSize = 5;
            end
            if nargin < 5
                elevationChangeRange = 1;
            end
            initX = xRange(1);
            initY = yRange(1);
            scaleX = scale;
            scaleY = scale;
            maxX = xRange(2);
            maxY = yRange(2);
            lengthX = length(initX:scaleX:maxX);
            lengthY = length(initY:scaleY:maxY);
            
            elevationMatrix = zeros(lengthX, lengthY);
            elevationMatrix(1,:) = randn(1, lengthY);
            xIndex = 2;
            for x = 1:lengthX-1
                randomizedZValue = randn(1, lengthY);
                randomizedZSign = randn(1, lengthY);
                randomizedZSign(randomizedZSign < 0.5) = -1;
                randomizedZSign(randomizedZSign >= 0.5) = 1;
                elevationsAcrossY = elevationMatrix(xIndex-1,:)+randomizedZValue.*randomizedZSign.*elevationChangeRange;
                elevationMatrix(xIndex,:) = elevationsAcrossY;
                xIndex = xIndex + 1;
            end
            
            % smooth elevations?
            K = (1/(filterSize*filterSize))*ones(filterSize);
            elevationMatrix = conv2(elevationMatrix,K,'same');
            
            slipMatrix = zeros(size(elevationMatrix));
            
            % save
            terrainName = sprintf(TerrainGenerator.nameRandom, maxX, maxY, filterSize, elevationChangeRange);
            fileAddress = sprintf(obj.saveFileAddress,terrainName);
            save(fileAddress, 'initX', 'initY', 'maxX', 'maxY', 'scaleX', 'scaleY', ...
            'elevationMatrix', 'slipMatrix');
        end
        function fileAddress = gen_different_left_right(obj, xRange, yRange, scale, amplitude, frequency, path)
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
            terrainName = sprintf(TerrainGenerator.nameLeftRight, maxX, maxY, amplitude, path.get_name());
            fileAddress = sprintf(obj.saveFileAddress,terrainName);
            save(fileAddress, 'initX', 'initY', 'maxX', 'maxY', 'scaleX', 'scaleY', ...
            'elevationMatrix', 'slipMatrix');
        end
        function fileAddress = gen_halfsin(obj, xRange, yRange, scale, amplitude, frequency, path)
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
            terrainName = sprintf(TerrainGenerator.nameHalfSin, maxX, maxY, amplitude, frequency);
            fileAddress = sprintf(obj.saveFileAddress,terrainName);
            save(fileAddress, 'initX', 'initY', 'maxX', 'maxY', 'scaleX', 'scaleY', ...
            'elevationMatrix', 'slipMatrix');
        end
    end
    methods(Static)
        function name = get_terrain_name(fromPath, path, type, varargin)
            if nargin < 2
                fromPath = false;
            end
            mapValues = cell(1,size(TerrainGenerator.argNames,2));
            if nargin > 3
                mapValues = varargin;
            end
            args = {};
            map = containers.Map(TerrainGenerator.argNames, mapValues);
            if fromPath == true
                [width, height] = TerrainGenerator.get_terrain_size_from_path(path);
            else
                width = map('width');
                height = map('height');
            end
            if strcmp(type, 'flat')
                name = TerrainGenerator.nameFlat;
            elseif strcmp(type, 'ramp')
                name = TerrainGenerator.nameRamp;
            elseif strcmp(type, 'sin')
                name = TerrainGenerator.nameSin;
                args = {map('amplitude'), map('frequency')};
            elseif strcmp(type, 'random')
                name = TerrainGenerator.nameRandom;
                args = {map('elevationChangeRange'),map('randFilterSize')};
            elseif strcmp(type, 'leftright')
                name = TerrainGenerator.nameLeftRight;
                args = {map('amplitude'), path.get_name()};
            elseif strcmp(type, 'halfsin')
                name = TerrainGenerator.nameHalfSin;
                args = {map('amplitude'), map('frequency'), path.get_name()};
            else
                disp('error');
                return;
            end
            name = sprintf(name, width, height, args{:}); 
        end
        function [terrainType, xRange, yRange, extraArgs] = get_data_from_string(input)
            args = split(input, '/');
            args = split(args{end}, TerrainGenerator.delimiter);
            args{end} = args{end}(1:end-4);
            terrainType = args{2};
            % shared
            xRange = [0 str2double(args{3})];
            yRange = [0 str2double(args{4})];
            extraArgs = {};
            if size(args, 2) > 4
                extraArgs = args{5:end};
            end
        end
        function [width, height] = get_terrain_size_from_path(path)
            [width, height, maxY, minY] = path.get_size();
            width = width + TerrainGenerator.terrainPadding(1);
            height = maxY;
            height = height + TerrainGenerator.terrainPadding(2);
        end
    end
end