classdef PathGenerator
    properties(Constant)
        % names
        delimiter = '_';
        % vars: startX, startY, endX, endY
        nameLine = 'line_%d_%d_%d_%d';
        nameCircle = 'circ_%d_%d_%d_%d';
        nameHalfCircle = 'halfcirc_%d_%d_%d_%d';
        nameHalfCircle2 = 'halfcirc2_%d_%d_%d_%d';
        % vars: startX, startY, endX, endY, amplitude
        nameSin = 'sin_%d_%d_%d_%d_%.3f';
        % args
        argNames = {'amplitude'};
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
            startPoint = [startX startY];
            endPoint = [endX endY];
            stepSize = 10;
            %pathRequiredArgs = {startPoint, endPoint, stepSize};
            if strcmp(pathType, 'line')
                fileAddress = obj.gen_line(startPoint, endPoint, stepSize);
            elseif strcmp(pathType, 'straight')
                endPoint(2) = startY;
                fileAddress = obj.gen_line(startPoint, endPoint, stepSize);
            elseif strcmp(pathType, 'halfcirc')
                sign = 1;
                fileAddress = obj.gen_halfcircle(startPoint, endPoint, stepSize, sign);
            elseif strcmp(pathType, 'halfcirc2')
                sign = -1;
                fileAddress = obj.gen_halfcircle(startPoint, endPoint, stepSize, sign);
            elseif strcmp(pathType, 'circ')
                fileAddress = obj.gen_diagonal(startPoint, endPoint, stepSize);
            elseif strcmp(pathType, 'sin')
                amplitude = varargin{1};
                fileAddress = obj.gen_sin(startPoint, endPoint, stepSize, amplitude);
            end
        end
        %% Generator Functions
        function fileAddress = gen_line(obj, startPoint, endPoint, stepSize)
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
            pathName = sprintf(PathGenerator.nameLine, startX, startY, endX, endY);
            fileAddress = sprintf(obj.saveFileAddress, pathName);
            save(fileAddress, 'pathPoints');
        end
        function fileAddress = gen_halfcircle(obj, startPoint, endPoint, stepSize, sign)
            startX = startPoint(1);
            startY = startPoint(2);
            endX = endPoint(1);
            endY = startY;
            
            diameter = abs(endX - startX);
            radius = diameter/2;
            radius_squared = radius.^2;
            
            pathGenName = PathGenerator.nameHalfCircle;
            if sign < 0
                pathGenName = PathGenerator.nameHalfCircle2;
                startY = radius+startY;
            end
            
            pathLength = ceil(pi*radius);
            pathX = linspace(startX, endX, pathLength);
            pathPoints = zeros(pathLength,2);
            for i=1:pathLength
                x = pathX(i)-startX;
                pathY = sqrt(radius_squared - (x-radius)^2);
                pathY = sign*pathY;
                pathY = pathY + startY;
                pathPoints(i,:) = [pathX(i) pathY];
            end
            pathName = sprintf(pathGenName, startX, startY, endX, endY);
            fileAddress = sprintf(obj.saveFileAddress, pathName);
            save(fileAddress, 'pathPoints');
        end
        function fileAddress = gen_sin(obj, startPoint, endPoint, stepSize, amplitude)
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
            pathName = sprintf(PathGenerator.nameSin, startX, startY, endX, endY, ...
                amplitude);
            fileAddress = sprintf(obj.saveFileAddress, pathName);
            save(fileAddress, 'pathPoints');
        end
    end
    methods(Static)
        function name = get_path_name(type, pathStartX, pathStartY, pathEndX, pathEndY, varargin)
            name = '';
            mapValues = cell(1,size(PathGenerator.argNames,2));
            if nargin > 5
                mapValues = varargin;
            end
            args = {};
            map = containers.Map(PathGenerator.argNames, mapValues);
            if strcmp(type, 'line')
                name = PathGenerator.nameLine;
            elseif strcmp(type, 'straight')
                pathEndY = pathStartY;
                name = PathGenerator.nameLine;
            elseif strcmp(type, 'halfcirc')
                name = PathGenerator.nameHalfCircle;
            elseif strcmp(type, 'halfcirc2')
                name = PathGenerator.nameHalfCircle2;
            elseif strcmp(type, 'circ')
                name = PathGenerator.nameCircle;
            elseif strcmp(type, 'sin')
                name = PathGenerator.nameSin;
                args = {map('amplitude')};
            else
                disp('error');
                return;
            end
            name = sprintf(name, pathStartX, pathStartY, pathEndX, pathEndY, args{:}); 
        end
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
    end
end