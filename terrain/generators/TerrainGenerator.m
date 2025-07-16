classdef TerrainGenerator
    properties(Constant)
        savePath = 'input_terrain/terrain_%s.mat';
        delimiter = '_';
        terrainPadding = [100 100];
        requiredArgNames = {'width', 'height', 'cellsize'}
        % optional args
        extraArgNames = {'amplitude', 'frequency', ...
                'randFilterSize', 'elevationChangeRange'};
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
            fileAddress = obj.gen_from_options(false, '',terrainType, xRange, yRange, scale, extraArgs);
        end
        function fileAddress = gen_from_options(obj, genFromPath, path, terrainType, width, height, ...
                scale, varargin)
            if genFromPath == true
                [width, height] = TerrainGenerator.get_terrain_size_from_path(path);
            end
            xRange = [0 width];
            yRange = [0 height];
            terrain = TerrainGenerator.get_type(terrainType, obj.directory);
            terrainArgs = terrain.argNames;
            indices = zeros(size(terrainArgs));
            for i=1:size(terrainArgs,2)
                index = find(strcmp(obj.extraArgNames, terrainArgs(i)));
                indices(i) = index;
            end
            fileAddress = terrain.generate(xRange, yRange, scale, varargin{indices});
        end
        function name = get_name(obj, fromPath, path, type, varargin)
            if nargin < 2
                fromPath = false;
            end
            %map = obj.create_arg_map(varargin);
            allArgNames = [TerrainGenerator.requiredArgNames, TerrainGenerator.extraArgNames];
            mapValues = cell(size(allArgNames));
            if size(varargin,2) > 0
                mapValues = varargin;
            end
            disp(mapValues)
            map = containers.Map(allArgNames, mapValues);
            if fromPath == true
                [width, height] = TerrainGenerator.get_terrain_size_from_path(path);
            else
                width = map('width');
                height = map('height');
            end
            terrain = TerrainGenerator.get_type(type, obj.directory);
            args = values(map, terrain.argNames);
            name = sprintf(terrain.name, width, height, args{:});
            name = sprintf('%s_%s', obj.prefix, name);
        end
    end
    methods(Static)
        %{
        function map = create_arg_map(varargin)
            allArgNames = [TerrainGenerator.requiredArgNames, TerrainGenerator.extraArgNames];
            mapValues = cell(size(allArgNames));
            if nargin > 0
                mapValues = varargin{:};
            end
            map = containers.Map(allArgNames, mapValues);
        end
        %}
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
            % need to add path start offset
            startingPathPoint = path.pathPoints(1,:);
            disp(startingPathPoint)
            width = width + startingPathPoint(1);
            %height = height + startingPathPoint(2);
        end
        %% terrain options
        function terrain = get_type(type, directory)
            options = {'flat', 'ramp', 'sin', 'random', 'leftright', 'halfsin', 'perlin', 'mars'};
            terrains = {Flat(directory), Ramp(directory), Sin(directory), Random(directory), ...
                LeftRight(directory), LeftRightSin(directory), Perlin(directory)};
            for i=1:size(options,2)
                terrain = terrains{ismember(options,type)};
            end
            %{
            if strcmp(type, 'flat')
                terrain = Flat(directory);
            elseif strcmp(type, 'ramp')
                terrain = Ramp(directory);
            elseif strcmp(type, 'sin')
                terrain = Sin(directory);
            elseif strcmp(type, 'random')
                terrain = Random(directory);
            elseif strcmp(type, 'leftright')
                terrain = LeftRight(directory);
            elseif strcmp(type, 'halfsin')
                terrain = LeftRightSin(directory);
            else
                disp('error');
                return;
            end
            %}
        end
    end
end