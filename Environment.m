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
        %times;
        totalTime;
        timeCosts;
        %energies;
        totalPower;
        powerCosts;
        jointDistances;
        endPosDistances;
        timeJoints;
        timeWheels;
        %jointChanges;
        %wheelDistance;
        %coveredDistance;
        %velocities;
        % bot types
        botKeys;
        botMap;
        analyzedBotKeys;
    end
    methods
        function obj = Environment(outputEnvDirectory, loadFromSaved, inputPathDirectory, inputTerrainDirectory, ...
                pathName, terrainName, pathArgs, terrainArgs)
            disp("pathARgs")
            disp(pathArgs)
            disp("terrainArgs")
            disp(terrainArgs)
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
            obj.terrain = MatrixTerrain(terrainName, inputTerrainDirectory, loadFromSaved, terrainArgs);
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
            obj.totalTime = zeros(numBotTypes, 1);
            obj.totalPower = zeros(numBotTypes, 1);
            obj.timeCosts = {[],[],[]};
            obj.powerCosts = {[],[],[]};
            obj.jointDistances = {[],[],[]};
            obj.endPosDistances = {[],[],[]};
            obj.timeJoints = {[],[],[]};
            obj.timeWheels = {[],[],[]};
            %obj.jointChanges = zeros(numBotTypes, 1);
            %obj.coveredDistance = zeros(numBotTypes, 1);
            %obj.velocities = zeros(numBotTypes, 1);
            for i=1:length(obj.botKeys)
                currentBotKey = obj.botKeys{i};
                if any(strcmp(currentBotKey,botKeys))
                    fprintf('analyzing %s \n', currentBotKey);
                    bot = obj.get_bot_by_type(currentBotKey);
                    bot = bot.traverse_terrain(obj.terrain, obj.path, startX, maxIterations);
                    [timeCosts, jointDistances, endPosDistances, timeJoints, timeWheels] = bot.analyze_time(obj.terrain);
                    powerCosts = bot.analyze_power(obj.terrain);
                    %{
                    if isa(bot, 'rollBot')
                        %obj.wheelDistance = distance;
                        %obj.coveredDistance(i) = bot.stateHistory(end).basePosition(1) - bot.stateHistory(1).basePosition(1);
                    else
                        %obj.coveredDistance(i) = distance;
                    end
                    %}
                    obj.totalTime(i) = sum(timeCosts);
                    obj.timeCosts{i} = timeCosts;
                    %obj.jointChanges(i) = maxJointChanges;
                    %obj.velocities(i) = velocity;
                    obj.jointDistances{i} = jointDistances;
                    obj.endPosDistances{i} = endPosDistances;
                    obj.timeJoints{i} = timeJoints;
                    obj.timeWheels{i} = timeWheels;
                    obj.totalPower(i) = sum(powerCosts, 'all');
                    obj.powerCosts{i} = powerCosts;
                    obj.bots{obj.botMap(currentBotKey)} = bot;
                end
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
        function visualize_single_frame(obj, outputVisualsDirectory, robotType, stateNumberPercent, showVisual, varargin)
            robots = {robotType};
            if strcmp(robotType,'all')
                robots = {'pull', 'roll', 'walk'};
            end
            figureVisibility = 'off';
            if showVisual > 0
                figureVisibility = 'on';
            end
            
            for i=length(robots)
                robotType = robots{i};
                fprintf('displaying robotType %s \n', robotType);
                bot = obj.get_bot_by_type(robotType);
                
                % robot type params: roll, pull, walk
                % must have run analyze() first
                figure('visible',figureVisibility);
                
                terrainVisual = Visualizer.simulate_terrain(obj.terrain);
                xlabel('X');
                ylabel('Y');
                zlabel('Z');
                
                hold on
                pathVisual = Visualizer.simulate_path(obj.path);
                hold off
                
                hold on
                Visualizer.simulate_bot_single_frame(bot, stateNumberPercent, varargin{:});
                hold off
    
                % save figure here
                if ~isempty(outputVisualsDirectory)
                    pathVisualFileName = sprintf("%s/robot_%s_%s_%s_%0.2f.png", outputVisualsDirectory, obj.terrainName, obj.pathName, robotType, stateNumberPercent);
                    saveas(gcf,pathVisualFileName)
                end
            end
        end
        function visualize_terrain(obj, outputVisualsDirectory, varargin)            
            figure;
            terrainVisual = Visualizer.simulate_terrain(obj.terrain);
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            if ~isempty(outputVisualsDirectory)
                pathVisualFileName = sprintf("%s/%s.png", outputVisualsDirectory, obj.terrainName);
                saveas(gcf,pathVisualFileName)
            end
        end
        function visualize_path(obj, outputVisualsDirectory, varargin)
            figure;
            
            terrainVisual = Visualizer.simulate_terrain(obj.terrain);
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            
            hold on
            pathVisual = Visualizer.simulate_path(obj.path);
            hold off

            if ~isempty(outputVisualsDirectory)
                pathVisualFileName = sprintf("%s/%s_%s.png", outputVisualsDirectory, obj.terrainName, obj.pathName);
                saveas(gcf,pathVisualFileName)
            end
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

            addpath('transformations');
            addpath('robot/kinematics');
            addpath('robot/analyzer');
            addpath('robot/legged');
            addpath('robot/legged/step_strategies');
        end
        function saveAddress = get_saved_address(directory)
            saveAddress = string(directory) + "/" + string(Environment.prefix) + "_%s_%s.mat";
        end
        function env = get_saved_env(outputEnvDirectory, pathDirectory, terrainDirectory, pathName, terrainName, pathArgs, terrainArgs)
            Environment.load_paths();
            saveAddress = Environment.get_saved_address(outputEnvDirectory);

            if strcmp(pathName, '')
                pathGen = PathGenerator();
                pathName = pathGen.get_name(pathArgs{:});
                disp(pathName);
            end
            fromPath = false;
            path = Path(pathName, pathGen.directory, pathArgs);
            if size(terrainArgs, 2) > 0
                fromPath = terrainArgs{1};
            end
            if fromPath == true
                terrainArgs{2} = path;
            end
            if strcmp(terrainName, '')
                disp(terrainArgs{1});
                terrainGen = TerrainGenerator();
                terrainName = terrainGen.get_name(terrainArgs{:});
                disp(terrainName);
            end
            
            savedFile = load(sprintf(saveAddress, terrainName, pathName));
            env = savedFile.env;
        end
    end
end