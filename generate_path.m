addpath('terrain')
savePath = 'output/results_%s.mat';
terrainOptions = {'flat', 'sin', 'random'};
pathOptions = {'line', 'halfcirc', 'sin'};
botBodyLength = 30;

% terrain
xRange = [0 800];
yRange = [0 600];
scale = 5;
width = xRange(2)-xRange(1);
height = yRange(2)-yRange(1);
TerrainGenerator.gen_flat(xRange, yRange, scale);
terrainName = TerrainGenerator.get_terrain_name('flat', xRange(2), yRange(2));

%% PARAMS
pathNum = 3;
loadFromSaved = 0;
equalAxis = 0;

stepSize = 10;
startX = xRange(1) + botBodyLength; % 30
startX = 50;
startY = (yRange(2)+yRange(1))/2;   % 100
endX = 650;
endY = 400;
startPoint = [startX startY];
endPoint = [endX endY];
amplitude = 100;
frequency = 0.01;


pathType = pathOptions{pathNum};
if loadFromSaved == 0
    disp('generating new path');
    if strcmp(pathType, 'line')
        PathGenerator.gen_straight(startPoint, endPoint, stepSize);
    elseif strcmp(pathType, 'halfcirc')
        PathGenerator.gen_halfcircle(startPoint, endPoint, stepSize);
    elseif strcmp(pathType, 'sin')
        PathGenerator.gen_sin(startPoint, endPoint, stepSize, amplitude);
    else
        disp('error');
        return
    end
end

% visualize?
pathArgs = {{endX}, {endX, endY}, {startX, startY, endX, endY, amplitude, frequency}, {}, {}};
pathName = PathGenerator.get_path_name(pathType, pathArgs{pathNum}{:});
disp(pathName);
env = Environment(terrainName, savePath, pathName);
env.visualize('roll', 0.2, 0, -1, 'z', 1, equalAxis);
