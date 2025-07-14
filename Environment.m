classdef Environment
    properties(Constant)
        prefix = 'results';
    end
    properties
        outputEnvAddress;
        inputPathDirectory;
        inputTerrainDirectory;
        %directory = 'output';
        pathGenerator;
        terrainGenerator;
        genTerrainFromPath = true;
        terrainName;
        terrain;
        pathName;
        path;
        bots;
        % traverse range variables
        startX;
        maxIterations;
        % analysis variables
        times;
        energies;
        jointChanges;
        wheelDistance;
        coveredDistance;
        velocities;
        % bot types
        botKeys;
        botMap;
        analyzedBotKeys;
    end
    methods
        function obj = Environment(outputEnvDirectory, inputPathDirectory, inputTerrainDirectory, ...
                pathName, terrainName, pathArgs, terrainArgs)
            Environment.load_paths();
            obj.outputEnvAddress = Environment.get_saved_address(outputEnvDirectory);
            disp("ENV ******f sav  **** ==")
            disp(obj.outputEnvAddress);
            if nargin < 6
                pathArgs = {};
            end
            if nargin < 7
                terrainArgs = {};
            end
            
            %{
            pathRequiredArgs = {pathOptions{pathNum}, pathStartX, pathStartY, pathEndX, pathEndY};
            pathExtraArgs = {pathAmplitude};
            pathArgs = [pathRequiredArgs pathExtraArgs];
            terrainRequiredArgs = {genTerrainFromPath, pathName, terrainOptions{terrainNum}};
            terrainExtraArgs = {width, height, cellsize, terrainAmplitude, terrainFrequency, randFilterSize, elevationChangeRange};
            terrainArgs = [terrainRequiredArgs terrainExtraArgs];
            %}
            
            if strcmp(pathName, '')
                pathGen = PathGenerator(inputPathDirectory);
                pathName = pathGen.get_name(pathArgs{:});
                disp(pathName);
            end
            fromPath = false;
            obj.path = Path(pathName, inputPathDirectory, pathArgs);
            obj.pathName = obj.path.pathName;
            if size(terrainArgs, 2) > 0
                fromPath = terrainArgs{1};
            end
            if fromPath == true
                terrainArgs{2} = obj.path;
            end
            if strcmp(terrainName, '')
                disp(terrainArgs{1});
                addpath('terrain/generators')
                terrainGen = TerrainGenerator(inputTerrainDirectory);
                terrainName = terrainGen.get_name(terrainArgs{:});
                disp(terrainName);
            end
            obj.terrain = MatrixTerrain(terrainName, inputTerrainDirectory, terrainArgs);
            obj.terrainName = obj.terrain.terrainName;
            
            % setup bots
            botArgs = {obj.terrain,obj.path};
            obj.bots = {PullBot(botArgs{:}), RollBot(botArgs{:}), WalkBot(botArgs{:})};
            obj.botMap = containers.Map({'pull', 'roll', 'walk'}, [1, 2, 3]);
            obj.botKeys = keys(obj.botMap);
        end
        function bot = get_bot_by_type(obj, type)
            bot = obj.bots{obj.botMap(type)};
        end
        function obj = save_bot_by_type(obj, bot, type)
            obj.bots{obj.botMap(type)} = bot;
        end
        function obj = analyze(obj, robotTypes, startX, maxIterations)
            botKeys = robotTypes;
            obj.analyzedBotKeys = botKeys;
            if nargin < 2 || any(strcmp(robotTypes,'all'))
                botKeys = obj.botKeys;
            end
            %% Traverse Range
            obj.startX = startX;
            obj.maxIterations = maxIterations;
            %% Analysis
            numBotTypes = length(botKeys);
            obj.times = zeros(numBotTypes, 1);
            obj.energies = zeros(numBotTypes, 1);
            obj.jointChanges = zeros(numBotTypes, 1);
            obj.coveredDistance = zeros(numBotTypes, 1);
            obj.velocities = zeros(numBotTypes, 1);
            for i=1:numBotTypes
                fprintf('analyzing %s \n', botKeys{i});
                bot = obj.get_bot_by_type(botKeys{i});
                bot = bot.traverse_terrain(obj.terrain, obj.path, startX, maxIterations);
                [time, maxJointChanges, timeCosts, velocity, distance] = bot.analyze_time(obj.terrain);
                if isa(bot, 'rollBot')
                    obj.wheelDistance = distance;
                    obj.coveredDistance(i) = bot.stateHistory(end).basePosition(1) - bot.stateHistory(1).basePosition(1);
                else
                    obj.coveredDistance(i) = distance;
                end
                obj.times(i) = time;
                obj.jointChanges(i) = maxJointChanges;
                obj.velocities(i) = velocity;
                obj.energies(i) = bot.analyze_power(obj.terrain);
                obj.bots{obj.botMap(botKeys{i})} = bot;
            end
            
            % Save Results
            env = obj;
            resultsFileName = strcat(obj.terrainName, '_', obj.pathName);
            fileAddress = sprintf(obj.outputEnvAddress, resultsFileName);
            save(fileAddress, 'env');
            %save(sprintf(obj.savePathData, obj.terrainName), '-struct', 'obj');
        end
        function visualize(obj, robotType, varargin)
            if strcmp(robotType,'all')
                robotType = obj.botKeys{1};
            end
            
            fprintf('displaying robotType %s \n', robotType);
            bot = obj.get_bot_by_type(robotType);
            
            % robot type params: roll, pull, walk
            % must have run analyze() first
            figure;
            
            terrainVisual = Visualizer.simulate_terrain(obj.terrain);
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            
            hold on
            pathVisual = Visualizer.simulate_path(obj.path);
            hold off
            
            Visualizer.simulate_bot(bot, varargin{:});
        end
    end
    methods(Static)
        function load_paths()
            addpath('terrain');
            addpath('terrain/generators')
            addpath('path');
            addpath('path/generators');
            addpath('robot');
            addpath('visualization');
        end
        function saveAddress = get_saved_address(directory)
            saveAddress = string(directory) + "/" + string(Environment.prefix) + "_%s.mat";
        end
        function env = get_saved_env(outputEnvDirectory, pathDirectory, terrainDirectory, pathName, terrainName, pathArgs, terrainArgs)
            saveAddress = Environment.get_saved_address(outputEnvDirectory);
            fileName = terrainName; 
            savedFile = load(sprintf(saveAddress, fileName));
            env = savedFile.env;
        end
    end
end