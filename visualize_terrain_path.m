%% ===== SETUP ======
addpath('terrain');
addpath('terrain/generators');
pathDirectory = 'input_path'; 
terrainDirectory = 'input_terrain';
outputEnvDirectory = 'output2';
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

%% VISUAL PARAMS
showTerrainOnly = false;
outputVisualDirectory = "output_visuals";
saveVisualAsImage = true;
%% ===== TEST PARAMS ======
loadFromSaved = false; 
showVisual = true;
showPlots = false;
turn = 0.01;
%% bot settings
botNum = 2;
botStartX = -1;
maxIterations = 4;
%% path settings
pathNum = 3;
pathStartX = 100; 
pathStartY = 100;
pathEndX = 2100;
pathEndY = pathStartY + 2000;
pathAmplitude = 100;
%% terrain settings
terrainNum = 3;
genTerrainFromPath = true;
% custom, ignore if gen from path
width = pathEndX+100;
height = pathEndY+200;
cellsize = 5;
% sin
terrainAmplitude = 13;
terrainFrequency = 0.05;
% random
randFilterSize = 10;
elevationChangeRange = 14;
%terrainName = 'customTerrainName';
%% visualize settings
rate = 0.001;
startState = 1;
stopState = -1;
showAllMarkers = true;
showAxis = '';
showColorBar = 0;
equalAxis = true;




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

% Load environment
if loadFromSaved == true
    env = Environment.get_saved_env(outputEnvDirectory, '', loadFromSaved, pathDirectory, terrainDirectory, ...
        pathName, terrainName, pathArgs, terrainArgs);
else
    env = Environment(outputEnvDirectory, '', loadFromSaved, pathDirectory, terrainDirectory, ...
        pathName, terrainName, pathArgs, terrainArgs);
end


if ~saveVisualAsImage
    outputVisualDirectory = '';
end
% Launch terrain visual
if showTerrainOnly
    env.visualize_terrain(outputVisualDirectory);
else
    env.visualize_path(outputVisualDirectory);
end

