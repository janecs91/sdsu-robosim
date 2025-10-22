%% ===== SETUP ======
addpath('terrain');
addpath('terrain/generators');
pathDirectory = 'input_path'; 
terrainDirectory = 'input_terrain';
outputEnvDirectory = 'output_results_paper';
outputPlotDirectory = 'output_plots_auto';
outputVisualsDirectory = 'output_visuals2';
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
addpath('input_args/input_terrain_args/sin');
executionGroup = 55;
if executionGroup == 1
    pathArgInstances = {StraightPathArgs()};
    terrainArgInstances = {FlatTerrainArgs()};
elseif executionGroup == 11
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs()};
    terrainArgInstances = {FlatTerrainArgs()};
elseif executionGroup == 2
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs(), SinPathArgs(), HalfCirclePathArgs()};
    terrainArgInstances = {FlatTerrainArgs(), RampUpTerrainArgs(), RampDownTerrainArgs(), LeftRightTerrainArgs(), HalfSinTerrainArgs()};
elseif executionGroup == 3
    % error - fix
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs(), SinPathArgs(), HalfCirclePathArgs()};
    terrainArgInstances = {RandomTerrainArgs()};
elseif executionGroup == 4
    % error - fix
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs(), SinPathArgs(), HalfCirclePathArgs()};
    terrainArgInstances = {PerlinTerrainArgs()};
elseif executionGroup == 5
    % Sin group - easy (rerun this, group was changed)
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs(), SinPathArgs(), HalfCirclePathArgs()};
    terrainArgInstances = {SinEasyLowTerrainArgs(), SinEasyMedTerrainArgs, SinEasyHighTerrainArgs, SinMedLowTerrainArgs, ...
        SinMedMedTerrainArgs, SinMedHighTerrainArgs, ...
        SinBumpyLowTerrainArgs, SinBumpyLowXTerrainArgs, SinBumpyLowXXTerrainArgs,
        SinBumpyMedTerrainArgs, SinBumpyMedXTerrainArgs
        };
elseif executionGroup == 55
    % Sin group - small subset
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs(), SinPathArgs(), HalfCirclePathArgs()};
    terrainArgInstances = {SinBumpyLowTerrainArgs, SinBumpyXLowTerrainArgs, SinBumpyXXLowTerrainArgs, ...
        SinBumpyMedTerrainArgs, SinBumpyXMedTerrainArgs
        };
elseif executionGroup == 7
    % possible ramp amplitude tests
end

pathName = "";
terrainName = "";
% Load environment
for terrainArgIndex = 1:length(terrainArgInstances)
    terrainArgInstance = terrainArgInstances{terrainArgIndex};
    numPaths = length(pathArgInstances);
    timeTable = table('Size',[numPaths, 4],'VariableTypes',{'string', 'double', 'double', 'double'},'VariableNames',{'Path', 'Roll', 'Pull', 'Walk'});
    powerTable = table('Size',[numPaths, 4],'VariableTypes',{'string', 'double', 'double', 'double'},'VariableNames',{'Path', 'Roll', 'Pull', 'Walk'});
    for pathArgIndex = 1:numPaths
        % path
        pathArgInstance = pathArgInstances{pathArgIndex};
        pathArgs = pathArgInstance.getPathArgs();
        forTablePathName = erase(class(pathArgInstance), 'PathArgs');
        % terrain
        terrainArgInstance = terrainArgInstance.setTerrainSize(pathArgInstance.pathEndX+100, pathArgInstance.pathEndY+200);
        terrainArgs = terrainArgInstance.getTerrainArgs(pathName);
        forTableTerrainName = erase(class(terrainArgInstance), 'TerrainArgs');
        
        envName = strcat(class(terrainArgInstance), '_', class(pathArgInstance));
        % load file
        load(sprintf("%s/results_%s.mat", outputEnvDirectory, envName), 'env');
        % get bots
        %disp(env.totalTime);
        timeTable(pathArgIndex, 1) = {forTablePathName};
        timeTable(pathArgIndex, 2:end) = {env.totalTime(2) env.totalTime(1) env.totalTime(3)};
        powerTable(pathArgIndex, 1) = {forTablePathName};
        powerTable(pathArgIndex, 2:end) = {env.totalPower(2) env.totalPower(1) env.totalPower(3)};
    end
    disp(forTableTerrainName);
    timeTable
    powerTable
end
