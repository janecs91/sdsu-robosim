%% ===== SETUP ======
addpath('terrain');
addpath('terrain/generators');
pathDirectory = 'input_path'; 
terrainDirectory = 'input_terrain';
outputEnvDirectory = 'output_results_paper';
outputPlotDirectory = 'output_plots_auto';
outputVisualsDirectory = 'output_visuals';
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
loadFromSavedTerrain = true;
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
%equalAxis = true;


%% ===== SIMULATE =====
% System setup
addpath('input_args/input_path_args');
addpath('input_args/input_terrain_args');
addpath('input_args/input_terrain_args/sin');
executionGroup = 5;
if executionGroup == 1
    % test group
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs()};
    terrainArgInstances = {FlatTerrainArgs()};
elseif executionGroup == 2
    % main easy group - done
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs(), SinPathArgs(), HalfCirclePathArgs()};
    terrainArgInstances = {FlatTerrainArgs(), RampUpTerrainArgs(), RampDownTerrainArgs(), LeftRightTerrainArgs(), HalfSinTerrainArgs()};
elseif executionGroup == 3
    % error - fix
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs(), SinPathArgs(), HalfCirclePathArgs()};
    terrainArgInstances = {RandomTerrainArgs()};
    loadFromSavedTerrain = true;
elseif executionGroup == 33
    % error - fix
    pathArgInstances = {DiagonalPathArgs(), SinPathArgs(), HalfCirclePathArgs()};
    terrainArgInstances = {RandomTerrainArgs()};
    loadFromSavedTerrain = true;
elseif executionGroup == 4
    % done
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs(), SinPathArgs(), HalfCirclePathArgs()};
    terrainArgInstances = {PerlinTerrainArgs()};
    loadFromSavedTerrain = true;
elseif executionGroup == 5
    outputPlotDirectory = 'output_plots_auto_sin';
    % Sin group - fixed amp
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs(), SinPathArgs(), HalfCirclePathArgs()};
    % amp = 5 (low)
    %terrainArgInstances = {SinEasyLowTerrainArgs(), SinMedLowTerrainArgs(), SinBumpyLowTerrainArgs(), SinBumpyXLowTerrainArgs(), SinBumpyXXLowTerrainArgs};
    % amp - 13 (high)
    terrainArgInstances = {SinEasyHighTerrainArgs(), SinMedHighTerrainArgs(), SinBumpyHighTerrainArgs(), SinBumpyXHighTerrainArgs(), SinBumpyXXHighTerrainArgs};
elseif executionGroup == 51
    outputPlotDirectory = 'output_plots_auto_sin';
    % Sin group - fixed freq = 1.0
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs(), SinPathArgs(), HalfCirclePathArgs()};
    % freq = 0.01 (easy)
    % repeats - SinEasyLowTerrainArgs, SinEasyHighTerrainArgs
    %terrainArgInstances = {SinEasyAmp7TerrainArgs(), SinEasyMedTerrainArgs, SinEasyAmp12TerrainArgs()};
    % freq = 1.0 (bumpyxx)
    % repeats - SinBumpyXXLowTerrainArgs, SinBumpyXXHighTerrainArgs
    terrainArgInstances = {
        SinBumpyXXMedTerrainArgs, ...
        SinBumpyXXAmp7TerrainArgs, ...
        SinBumpyXXAmp12TerrainArgs, ...
        };
elseif executionGroup == 52
    outputPlotDirectory = 'output_plots_auto_sin';
    % Sin group - the rest (med, bumpy, bumpyx)
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs(), SinPathArgs(), HalfCirclePathArgs()};
    terrainArgInstances = {SinMedMedTerrainArgs, SinBumpyMedTerrainArgs, SinBumpyXMedTerrainArgs};
elseif executionGroup == 8
    % possible ramp amplitude tests
end

pathName = "";
terrainName = "";
% Load environment
for pathArgIndex = 1:length(pathArgInstances)
    pathArgInstance = pathArgInstances{pathArgIndex};
    disp(pathArgInstance)
    pathArgs = pathArgInstance.getPathArgs();
    disp(pathArgs)
    forPlotPathName = erase(class(pathArgInstance), 'PathArgs');
    for terrainArgIndex = 1:length(terrainArgInstances)
        terrainArgInstance = terrainArgInstances{terrainArgIndex};
        terrainArgInstance = terrainArgInstance.setTerrainSize(pathArgInstance.pathEndX+100, pathArgInstance.pathEndY+200);
        terrainArgs = terrainArgInstance.getTerrainArgs(pathName);
        forPlotTerrainName = erase(class(terrainArgInstance), 'TerrainArgs');
        envName = strcat(class(terrainArgInstance), '_', class(pathArgInstance));
        fprintf("Env Name: %s\n", envName);
        if loadFromSaved == true
            env = Environment.get_saved_env(outputEnvDirectory, envName, loadFromSaved, pathDirectory, terrainDirectory, ...
                pathName, terrainName, pathArgs, terrainArgs);
        else
            env = Environment(outputEnvDirectory, envName, loadFromSavedTerrain, pathDirectory, terrainDirectory, ...
                pathName, terrainName, pathArgs, terrainArgs);
        end
        env = env.analyze(botOptions{botNum}, botStartX, maxIterations);

        %% WIP - single frame visual
        %% To do: test this for all 3 robot types
        stateNumberPercent = 0.5;
        equalAxis = false;
        env.visualize_single_frame(outputVisualsDirectory, botOptions{botNum}{1}, stateNumberPercent, equalAxis, showVisual, showAllMarkers, showAxis, showColorBar);
        equalAxis = true;
        env.visualize_single_frame(outputVisualsDirectory, botOptions{botNum}{1}, stateNumberPercent, equalAxis, showVisual, showAllMarkers, showAxis, showColorBar);
        stateNumberPercent = 1;
        equalAxis = false;
        env.visualize_single_frame(outputVisualsDirectory, botOptions{botNum}{1}, stateNumberPercent, equalAxis, showVisual, showAllMarkers, showAxis, showColorBar);
        equalAxis = true;
        env.visualize_single_frame(outputVisualsDirectory, botOptions{botNum}{1}, stateNumberPercent, equalAxis, showVisual, showAllMarkers, showAxis, showColorBar);

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
