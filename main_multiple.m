%% ===== SETUP ======
addpath('terrain');
addpath('terrain/generators');
pathDirectory = 'input_path'; 
terrainDirectory = 'input_terrain';
outputEnvDirectory = 'output2';
outputPlotDirectory = 'output_plots_auto';
outputVisualsDirectory = 'output_visuals_run';
botOptions = {{'all'}, {'roll'}, {'pull'}, {'walk'}};
terrainOptions = {'flat', 'ramp', 'sin', 'random', 'leftright', 'halfsin',   'perlin', 'mars'};
pathOptions = {'straight', 'line', 'halfcirc', 'halfcirc2', 'sin'};

%% NEW AGENDA !!!!!!!!!!!!!!!!!!!!!!!!!!
%=====================

%% GRAPHS
% [metric] - Max joint change distance
% [metric] - Stridge length: end position, body
% [QoL] - Save graphs

%% 07.25
% - Fix RollBot base end position?
% - Fix axis
% - Fix load from env; possibly fix terrain load
% [low] Fix Visualizer - what about it?
% [low] Possibly fix Perlin?
% [med] Possibly optimize step adjuster height

%% TBD: Analysis quality of life improvements
%{
1. Figure out how to save all state history of experiments
2. Generate graphs or pictures of a specific state
3. Iterate over array or combination of experiments
%}

%=====================
% OLD agenda
% fix paths (sin), fix terrains ??? Is this still valid?
% improve perlin -> Maybe not needed anymore. Use any easy perlin
% fix height adjustment ? Is this valid?
% auto stop (find max iterations) -> What is this again?
% 1. look into slight deviation from path -- sin path on sin terrain
% 3. read sections 2 and 3 Unified Kinematics, make notes of questions
% 4. write ch 2 of thesis
% additional terrains: steplike, mars?

%% ===== TEST PARAMS ======
loadFromSaved = false; 
showVisual = false;
createPlots = true;
showPlots = false;
turn = 0.01;
%% bot settings
botNum = 1;
botStartX = -1;
maxIterations = 9999;
%% visualize settings
rate = 0.01;
startState = 1;
stopState = -1;
showAllMarkers = false;
showAxis = '';
showColorBar = 0;
equalAxis = false;


%% ===== SIMULATE =====
% System setup
addpath('input_args/input_path_args');
addpath('input_args/input_terrain_args');
runAll = false;
pathArgInstances = {StraightPathArgs()};
terrainArgInstances = {FlatTerrainArgs()};
if runAll
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs(), SinPathArgs(), HalfCirclePathArgs()};
    terrainArgInstances = {FlatTerrainArgs(), RampUpTerrainArgs(), RampDownTerrainArgs(), LeftRightTerrainArgs(), HalfSinTerrainArgs(), RandomTerrainArgs(), PerlinTerrainArgs()};
end

pathName = "";
terrainName = "";
% Load environment
for pathArgIndex = 1:length(pathArgInstances)
    pathArgInstance = pathArgInstances{pathArgIndex};
    disp(pathArgInstance)
    pathArgs = pathArgInstance.getPathArgs();
    disp(pathArgs)
    for terrainArgIndex = 1:length(terrainArgInstances)
        terrainArgInstance = terrainArgInstances{terrainArgIndex};
        terrainArgInstance = terrainArgInstance.setTerrainSize(pathArgInstance.pathEndX+100, pathArgInstance.pathEndY+200);
        terrainArgs = terrainArgInstance.getTerrainArgs(pathName);
        if loadFromSaved == true
            env = Environment.get_saved_env(outputEnvDirectory, loadFromSaved, pathDirectory, terrainDirectory, ...
                pathName, terrainName, pathArgs, terrainArgs);
        else
            env = Environment(outputEnvDirectory, loadFromSaved, pathDirectory, terrainDirectory, ...
                pathName, terrainName, pathArgs, terrainArgs);
        end
        env = env.analyze(botOptions{botNum}, botStartX, maxIterations);

        %% WIP - single frame visual
        %% To do: test this for all 3 robot types
        stateNumberPercent = 0.5;
        env.visualize_single_frame(outputVisualsDirectory, botOptions{botNum}{1}, stateNumberPercent, showVisual, showAllMarkers, showAxis, showColorBar, equalAxis);
        stateNumberPercent = 1;
        env.visualize_single_frame(outputVisualsDirectory, botOptions{botNum}{1}, stateNumberPercent, showVisual, showAllMarkers, showAxis, showColorBar, equalAxis);

        %% Plot
        if createPlots
            addpath('plot');
            plotter = BotPlotter(env, outputPlotDirectory);
            plotter.plot_time_per_distance(showPlots);
            plotter.plot_power_per_distance(showPlots);
        end
    end
end

% System variables
% To keep in workspace
%{
keyOrder = env.botKeys;
totalTimes = env.totalTime;
totalPowers = env.totalPower;
path = env.path;
pathPoints = path.pathPoints;
terrain = env.terrain;
bots = env.bots;
pullBot = env.bots{1};
%}

% System display data
%{
disp(keyOrder)
disp('total times');
disp(totalTimes);
disp('total powers');
disp(totalPowers);
%}

% Launch visualization (if enabled)
%{
if showVisual == true
    env.visualize(botOptions{botNum}{1}, ...
        rate, startState, stopState, showAllMarkers, showAxis, showColorBar, equalAxis);
end
%}
