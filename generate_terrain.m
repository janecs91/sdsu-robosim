addpath('terrain')
addpath('terrain/generators')
savePath = 'output/results_%s.mat';
terrainOptions = {'flat', 'sin', 'random'};

%% PARAMS
terrainNum = 3;
loadFromSaved = 0;
equalAxis = 1;
xRange = [0 560];
yRange = [0 500];
scale = 5;
% sin
amplitude = 5;
frequency = 0.1;
% random
convFilterSize = 5;
elevationChangeRange = 5;

terrainType = terrainOptions{terrainNum};
if loadFromSaved == 0
    disp('generating new terrain');
    if strcmp(terrainType, 'flat')
        Flat.generate(xRange, yRange, scale);
    elseif strcmp(terrainType, 'sin')
        Sin.generate(xRange, yRange, scale, amplitude, frequency);
    elseif strcmp(terrainType, 'random')
        Random.generate(xRange, yRange, scale, convFilterSize, elevationChangeRange);
    else
        disp('error');
        return
    end
end

% visualize?
terrainArgs = {0, {amplitude, frequency}, {randFilterSize}};
terrainName = TerrainGenerator.get_terrain_name(terrainType, xRange(2), terrainArgs{terrainNum}{:});
disp(terrainName);
env = Environment(terrainName, savePath);
env.visualize('pull', 0.2, 0, -1, '', 1, equalAxis);
