%% ===== SETUP ======
addpath('terrain');
addpath('terrain/generators');
pathDirectory = 'input_path'; 
terrainDirectory = 'input_terrain';
outputEnvDirectory = 'output2';
outputPlotDirectory = 'output_plots';
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
showVisual = true;
createPlots = false;
showPlots = false;
turn = 0.01;
%% bot settings
botNum = 3;
botStartX = -1;
maxIterations = 9999;
%% path settings
% 1, 2, 3, 5
pathNum = 2;
pathStartX = 100; 
pathStartY = 100;
pathEndX = 2150;
pathEndY = pathStartY + 1000;
pathAmplitude = 100;
%% terrain settings
terrainNum = 4;
genTerrainFromPath = true;
% custom, ignore if gen from path
width = pathEndX+100;
height = pathEndY+200;
cellsize = 5;
% sin
terrainAmplitude = 10;
terrainFrequency = 0.1;
% random
randFilterSize = 100;
elevationChangeRange = 300;
%terrainName = 'customTerrainName';
%% visualize settings
rate = 0.01;
startState = 1;
stopState = -1;
showAllMarkers = true;
showAxis = '';
showColorBar = 0;
equalAxis = false;


%% ===== SIMULATE =====
% System setup
pathName = '';
terrainName = '';
pathRequiredArgs = {pathOptions{pathNum}, pathStartX, pathStartY, pathEndX, pathEndY};
pathExtraArgs = {pathAmplitude};
pathArgs = [pathRequiredArgs pathExtraArgs];
terrainRequiredArgs = {genTerrainFromPath, pathName, terrainOptions{terrainNum}};
terrainExtraArgs = {width, height, cellsize, terrainAmplitude, terrainFrequency, randFilterSize, elevationChangeRange};
terrainArgs = [terrainRequiredArgs terrainExtraArgs];

envName = '';
% Load environment
if loadFromSaved == true
    env = Environment.get_saved_env(outputEnvDirectory, envName, loadFromSaved, pathDirectory, terrainDirectory, ...
        pathName, terrainName, pathArgs, terrainArgs);
else
    env = Environment(outputEnvDirectory, envName, loadFromSaved, pathDirectory, terrainDirectory, ...
        pathName, terrainName, pathArgs, terrainArgs);
    try
        env = env.analyze(botOptions{botNum}, botStartX, maxIterations);
    catch ME
        disp("Error during robot analysis!")
        disp(ME)
    end
end

% System variables
% To keep in workspace
keyOrder = env.botKeys;
totalTimes = env.totalTime;
totalPowers = env.totalPower;
path = env.path;
pathPoints = path.pathPoints;
terrain = env.terrain;
bots = env.bots;
pullBot = env.bots{1};

% System display data
disp(keyOrder)
disp('total times');
disp(totalTimes);
disp('total powers');
disp(totalPowers);

% Launch visualization (if enabled)
if showVisual == true
    env.visualize(botOptions{botNum}{1}, ...
        rate, startState, stopState, showAllMarkers, showAxis, showColorBar, equalAxis);
end

%% Plot
if createPlots
    addpath('plot');
    plotter = BotPlotter(env, outputPlotDirectory);
    plotter.plot_time_per_distance(showPlots);
    plotter.plot_power_per_distance(showPlots);
end


