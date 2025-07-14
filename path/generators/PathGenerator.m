classdef PathGenerator
    properties(Constant)
        delimiter = '_';
        % args
        extraArgNames = {'amplitude'};
    end
    properties
        saveFileAddress;
        directory = 'input_path';
        prefix = 'path';
        robotSize = 50;
    end
    methods
        function obj = PathGenerator(directory)
            obj.saveFileAddress = string(directory) + "/" + string(obj.prefix) + "_%s.mat";
        end
        function saveFileAddress = get_save_address(obj)
            saveFileAddress = obj.saveFileAddress;
        end
        %% Helper Generator Functions
        function fileAddress = gen_from_string(obj, input)
            % shared 
            [pathType, startX, startY, endX, endY, extraArgs] = PathGenerator.get_data_from_string(input);
            stepSize = 10;
            % custom
            fileAddress = obj.gen_from_options(pathType, startX, startY, endX, endY, stepSize, extraArgs);
        end
        function fileAddress = gen_from_options(obj, pathType, startX, startY, endX, endY, varargin)
            %pathRequiredArgs = {startPoint, endPoint, stepSize};
            startPoint = [startX startY];
            endPoint = [endX endY];
            stepSize = 10;
            sprintf("%d %d %d %d\n", startPoint, endPoint)
            
            % type dependent extras
            path = PathGenerator.get_type(pathType, obj.directory);
            args = {};
            if strcmp(pathType, 'straight')
                endPoint(2) = startY;
            elseif strcmp(pathType, 'halfcirc')
                sign = 1;
                args = {sign};
            elseif strcmp(pathType, 'halfcirc2')
                sign = -1;
                args = {sign};
            elseif strcmp(pathType, 'sin')
                %amplitude = varargin{1};
                args = {varargin{1}};
            end
            
            fileAddress = path.generate(startPoint, endPoint, stepSize, args{:});
        end
        function name = get_name(obj, type, pathStartX, pathStartY, pathEndX, pathEndY, varargin)
            %map = PathGenerator.create_arg_map(varargin);
            mapValues = cell(size(PathGenerator.extraArgNames));
            if size(varargin,2) > 0
                mapValues = varargin;
            end
            map = containers.Map(PathGenerator.extraArgNames, mapValues);
            %width = abs(pathEndX-pathStartX);
            %height = abs(pathEndY-pathStartY);
            
            path = PathGenerator.get_type(type, obj.directory);
            args = values(map, path.argNames);
            if path.isSameStartYEndY
                pathEndY = pathStartY;
            end
            name = sprintf(path.name, pathStartX, pathStartY, pathEndX, pathEndY, args{:});
            name = sprintf('%s_%s', obj.prefix, name);
        end
    end
    methods(Static)
        %{
        function map = create_arg_map(varargin)
            mapValues = cell(size(PathGenerator.extraArgNames));
            if nargin > 0
                mapValues = varargin{:};
            end
            map = containers.Map(PathGenerator.extraArgNames, mapValues);
        end
        %}
        function [pathType, startX, startY, endX, endY, extraArgs] = get_data_from_string(input)
            args = split(input, '/');
            args = args(end);
            args = split(args{end}, PathGenerator.delimiter);
            disp(args{end})
            args{end} = split(args{end}, '.mat');
            args{end} = args{end}{1};
            disp("PATH NAME")
            disp(args)
            pathType = args{1};
            % shared 
            startX = str2double(args{2});
            startY = str2double(args{3});
            endX = str2double(args{4});
            endY = str2double(args{5});
            extraArgs = {};
            if size(args, 2) > 5
                extraArgs = args{6:end};
            end
        end
        %% path options
        function path = get_type(type, directory)
            %directory = obj.directory;
            options = {'straight', 'line', 'halfcirc', 'halfcirc2', 'sin'};
            paths = {LinePath(directory), LinePath(directory), HalfCirclePath(directory), ...
                HalfCirclePath(directory), SinPath(directory)};
            for i=1:size(options,2)
                %index = find(strcmp(terrainOptions, type));
                path = paths{ismember(options,type)};
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