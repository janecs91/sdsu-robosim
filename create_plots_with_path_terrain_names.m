%% ===== SETUP ======
addpath('terrain');
addpath('terrain/generators');
pathDirectory = 'input_path'; 
terrainDirectory = 'input_terrain';
outputEnvDirectory = 'output_results_paper';
outputPlotDirectory = 'output_plots_renamed_sin';
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
Environment.load_paths()
addpath('input_args/input_path_args');
addpath('input_args/input_terrain_args');
executionGroup = 5;
if executionGroup == 1
    pathArgInstances = {StraightPathArgs()};
    terrainArgInstances = {FlatTerrainArgs()};
elseif executionGroup == 11
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs()};
    terrainArgInstances = {FlatTerrainArgs()};
elseif executionGroup == 2
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs(), SinPathArgs(), HalfCirclePathArgs()};
    terrainArgInstances = {FlatTerrainArgs(), RampUpTerrainArgs(), RampDownTerrainArgs(), LeftRightTerrainArgs(), HalfSinTerrainArgs()};
elseif executionGroup == 22
    % almost all except sin and random
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs(), SinPathArgs(), HalfCirclePathArgs()};
    terrainArgInstances = {FlatTerrainArgs(), RampUpTerrainArgs(), RampDownTerrainArgs(), LeftRightTerrainArgs(), HalfSinTerrainArgs(), ...
        PerlinTerrainArgs()
        };
elseif executionGroup == 3
    % error - fix
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs(), SinPathArgs(), HalfCirclePathArgs()};
    terrainArgInstances = {RandomTerrainArgs()};
elseif executionGroup == 4
    % error - fix
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs(), SinPathArgs(), HalfCirclePathArgs()};
    terrainArgInstances = {PerlinTerrainArgs()};
elseif executionGroup == 5
    % Sin group - almost all types
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs(), SinPathArgs(), HalfCirclePathArgs()};
    terrainArgInstances = {SinEasyLowTerrainArgs(), SinEasyMedTerrainArgs, SinEasyHighTerrainArgs, SinMedLowTerrainArgs, ...
        SinMedMedTerrainArgs, SinMedHighTerrainArgs, ...
        SinBumpyLowTerrainArgs, SinBumpyLowXTerrainArgs, SinBumpyLowXXTerrainArgs, ...
        SinBumpyMedTerrainArgs, SinBumpyMedXTerrainArgs, ...
        };
elseif executionGroup == 7
    % possible ramp amplitude tests
end

pathName = "";
terrainName = "";
% Load environment
numPaths = length(pathArgInstances);
for pathArgIndex = 1:numPaths
    % path
    pathArgInstance = pathArgInstances{pathArgIndex};
    pathArgs = pathArgInstance.getPathArgs();
    forPlotPathName = erase(class(pathArgInstance), 'PathArgs');
    for terrainArgIndex = 1:length(terrainArgInstances)
        terrainArgInstance = terrainArgInstances{terrainArgIndex};
        % terrain
        terrainArgInstance = terrainArgInstance.setTerrainSize(pathArgInstance.pathEndX+100, pathArgInstance.pathEndY+200);
        terrainArgs = terrainArgInstance.getTerrainArgs(pathName);
        forPlotTerrainName = erase(class(terrainArgInstance), 'TerrainArgs');
        
        envName = strcat(class(terrainArgInstance), '_', class(pathArgInstance));
        % load file
        load(sprintf("%s/results_%s.mat", outputEnvDirectory, envName), 'env');
        % get bots
        %disp(env.totalTime);
        %% Plot
        if createPlots
            addpath('plot');
            plotter = BotPlotter(env, outputPlotDirectory);
            plotter.plot_time_per_distance(showPlots, forPlotPathName, forPlotTerrainName);
            plotter.plot_power_per_distance(showPlots, forPlotPathName, forPlotTerrainName);
        end
    end
    disp(forPlotPathName);
end
