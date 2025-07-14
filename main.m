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
% MINIMIZE OR CLEAN UP PRINT STATEMENTS
% Go through program step by step
% Review math and code
% Add or clean up comments. Use Copilot to help if needed

%% Potential fixes and ideas
% Back legs are planning to move too far ahead
% Added safetyValue* factor to back leg max. Review.

%% TBD: Analysis quality of life improvements
%{
1. Figure out how to save all state history of experiments
2. Generate graphs or pictures of a specific state
3. Iterate over array or combination of experiments
%}

%% ADDITIONAL ANALYSIS FEATURES????
%{ 
Interesting things to add: !!!!!!!!!!!!!!!!!
Get base position distance for each leg (size of body vector differences in
pulling vs walking) - does pulling have a bigger body vector?
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
turn = 0.01;
%% bot settings
botNum = 4;
botStartX = -1;
maxIterations = 100;
%% path settings
pathNum = 5;
pathStartX = 50; 
pathStartY = 100;
pathEndX = 500;
pathEndY = pathStartY + 30;
pathAmplitude = 100;
%% terrain settings
terrainNum = 1;
genTerrainFromPath = true;
% custom, ignore if gen from path
width = pathEndX+50;
height = pathEndY+200;
cellsize = 5;
% sin
terrainAmplitude = 20;
terrainFrequency = 0.01;
% random
randFilterSize = 10;
elevationChangeRange = 10;
%terrainName = 'customTerrainName';
%% visualize settings
rate = 0.001;
startState = 1;
stopState = 2;
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

% Load environment
if loadFromSaved == true
    env = Environment.get_saved_env(outputEnvDirectory, pathDirectory, terrainDirectory, ...
        pathName, terrainName, pathArgs, terrainArgs);
else
    env = Environment(outputEnvDirectory, pathDirectory, terrainDirectory, ...
        pathName, terrainName, pathArgs, terrainArgs);
    env = env.analyze(botOptions{botNum}, botStartX, maxIterations);
end

% System variables
% To keep in workspace
keyOrder = env.botKeys;
times = env.times;
energies = env.energies;
path = env.path;
pathPoints = path.pathPoints;
terrain = env.terrain;
bots = env.bots;
pullBot = env.bots{1};

% System display data
disp(keyOrder)
disp('times');
disp(times);
disp('energies');
disp(energies);

% Launch visualization (if enabled)
if showVisual == true
    env.visualize(botOptions{botNum}{1}, ...
        rate, startState, stopState, showAllMarkers, showAxis, showColorBar, equalAxis);
end