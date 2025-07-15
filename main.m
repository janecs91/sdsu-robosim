%% ===== SETUP ======
addpath('terrain');
addpath('terrain/generators');
pathDirectory = 'input_path'; 
terrainDirectory = 'input_terrain';
outputEnvDirectory = 'output';
botOptions = {{'all'}, {'roll'}, {'pull'}, {'walk'}};
terrainOptions = {'flat', 'ramp', 'sin', 'random', 'leftright', 'halfsin', 'perlin', 'mars'};
pathOptions = {'straight', 'line', 'halfcirc', 'halfcirc2', 'sin'};

%% Meeting MWF @ 1:00 PM

% agenda
% Code & Structure
% fix paths (sin), fix terrains
% improve perlin
% fix height adjustment
% auto stop (find max iterations)

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
pathEndX = 1000;
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
stopState = -1;
showAxis = '';
showColorBar = 0;
equalAxis = false;


%% ===== SIMULATE =====
pathName = '';
terrainName = '';
pathRequiredArgs = {pathOptions{pathNum}, pathStartX, pathStartY, pathEndX, pathEndY};
pathExtraArgs = {pathAmplitude};
pathArgs = [pathRequiredArgs pathExtraArgs];
terrainRequiredArgs = {genTerrainFromPath, pathName, terrainOptions{terrainNum}};
terrainExtraArgs = {width, height, cellsize, terrainAmplitude, terrainFrequency, randFilterSize, elevationChangeRange};
terrainArgs = [terrainRequiredArgs terrainExtraArgs];

if loadFromSaved == true
    env = Environment.get_saved_env(outputEnvDirectory, pathDirectory, terrainDirectory, ...
        pathName, terrainName, pathArgs, terrainArgs);
else
    env = Environment(outputEnvDirectory, pathDirectory, terrainDirectory, ...
        pathName, terrainName, pathArgs, terrainArgs);
    env = env.analyze(botOptions{botNum}, botStartX, maxIterations);
end

% keep in workspace
keyOrder = env.botKeys;
times = env.times;
energies = env.energies;
path = env.path;
pathPoints = path.pathPoints;
terrain = env.terrain;
bots = env.bots;
pullBot = env.bots{1};

% display
disp(keyOrder)
disp('times');
disp(times);
disp('energies');
disp(energies);

if showVisual == true
    env.visualize(botOptions{botNum}{1}, ...
        rate, startState, stopState, showAxis, showColorBar, equalAxis);
end