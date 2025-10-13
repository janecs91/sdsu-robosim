%% ===== SETUP ======
addpath('terrain');
addpath('terrain/generators');
pathDirectory = 'input_path'; 
terrainDirectory = 'input_terrain';
outputEnvDirectory = 'output_results_paper';
outputPlotDirectory = 'output_plots_auto_sin';
outputVisualsDirectory = 'output_visuals_run_sin';
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


% todo: finish plotting sin trends
% fix height adjustment for legged bot

%% ===== SIMULATE =====
% System setup
addpath('input_args/input_path_args');
addpath('input_args/input_terrain_args');
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
        SinBumpyLowTerrainArgs, SinBumpyLowXTerrainArgs, SinBumpyLowXXTerrainArgs};
elseif executionGroup == 55
    % Sin group - easy (rerun this, group was changed)
    pathArgInstances = {StraightPathArgs()};
    terrainArgInstances = {SinEasyLowTerrainArgs(), SinEasyMedTerrainArgs, SinMedLowTerrainArgs, ...
        SinBumpyLowTerrainArgs, SinBumpyLowXTerrainArgs, SinBumpyLowXXTerrainArgs};
elseif executionGroup == 6
    % Sin group - harder
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs(), SinPathArgs(), HalfCirclePathArgs()};
    terrainArgInstances = {SinMedMedTerrainArgs, SinMedHighTerrainArgs, SinBumpyMedTerrainArgs, SinBumpyMedXTerrainArgs};
elseif executionGroup == 7
    % possible ramp amplitude tests
end

pathName = "";
terrainName = "";
% Load environment
for pathArgIndex = 1:length(pathArgInstances)
    pathArgInstance = pathArgInstances{pathArgIndex};
    pathArgs = pathArgInstance.getPathArgs();
    forTablePathName = erase(class(pathArgInstance), 'PathArgs');
    numTerrains = length(terrainArgInstances);
    amplitudes = zeros(numTerrains,1);
    frequencies = zeros(numTerrains,1);
    times = zeros(3, numTerrains);
    powers = zeros(3, numTerrains);
    for terrainArgIndex = 1:numTerrains
        terrainArgInstance = terrainArgInstances{terrainArgIndex};
        terrainArgInstance = terrainArgInstance.setTerrainSize(pathArgInstance.pathEndX+100, pathArgInstance.pathEndY+200);
        terrainArgs = terrainArgInstance.getTerrainArgs(pathName);
        forTableTerrainName = erase(class(terrainArgInstance), 'TerrainArgs');
        envName = strcat(class(terrainArgInstance), '_', class(pathArgInstance));
        % load file
        Environment.load_paths()
        load(sprintf("%s/results_%s.mat", outputEnvDirectory, envName), 'env');
        
        amplitudes(terrainArgIndex) = terrainArgInstance.terrainAmplitude;
        frequencies(terrainArgIndex) = terrainArgInstance.terrainFrequency;
        times(:, terrainArgIndex) = env.totalTime(:);
        powers(:, terrainArgIndex) = env.totalPower(:);
    end
    disp(forTableTerrainName);
    disp(times)
    fixedAmplitude = 5;
    fixedFrequency = 0.01;
    %% 2d time plot
    % amplitude
    desired2DAmplitudeIndices = find(frequencies==fixedFrequency);
    disp("Indices")
    disp(desired2DAmplitudeIndices)
    figure;
    plot(amplitudes(desired2DAmplitudeIndices)', times(:,desired2DAmplitudeIndices))
    xlabel("Amplitude")
    ylabel("Time")
    title(sprintf("Plot Time Trend for Sin Terrain (fixed frequency=%.2d)", fixedFrequency))
    legend({'pull','roll','walk'},'Location','northeast')
    % frequency
    desired2DFrequencyIndices = find(amplitudes==fixedAmplitude);
    figure;
    plot(frequencies(desired2DFrequencyIndices)', times(:,desired2DFrequencyIndices))
    xlabel("Frequency")
    ylabel("Time")
    title(sprintf("Plot Time Trend for Sin Terrain (fixed amplitude=%.2d)", fixedAmplitude))
    legend({'pull','roll','walk'},'Location','northeast')
    %% 3d time plot with both amplitude & frequency
    figure;
    plot3(amplitudes, frequencies, times(:,:))
    xlabel("Amplitude")
    ylabel("Frequency")
    zlabel("Time")
    title("Plot Time Trend for Sin Terrain")
    legend({'pull','roll','walk'},'Location','northeast')

    %% 2d power plot
    % amplitude
    desired2DAmplitudeIndices = find(frequencies==fixedFrequency);
    figure;
    plot(amplitudes(desired2DAmplitudeIndices)', powers(:,desired2DAmplitudeIndices))
    xlabel("Amplitude")
    ylabel("Power")
    title(sprintf("Plot Time Trend for Sin Terrain (fixed frequency=%.2d)", fixedFrequency))
    legend({'pull','roll','walk'},'Location','northeast')
    % frequency
    desired2DFrequencyIndices = find(amplitudes==fixedAmplitude);
    figure;
    plot(frequencies(desired2DFrequencyIndices)', powers(:,desired2DFrequencyIndices))
    xlabel("Frequency")
    ylabel("Power")
    title(sprintf("Plot Time Trend for Sin Terrain (fixed amplitude=%.2d)", fixedAmplitude))
    legend({'pull','roll','walk'},'Location','northeast')
    %% 3d power plot with both amplitude & frequency
    figure;
    plot3(amplitudes, frequencies, powers(:,:))
    xlabel("Amplitude")
    ylabel("Frequency")
    zlabel("Power")
    title("Plot Power Trend for Sin Terrain")
    legend({'pull','roll','walk'},'Location','northeast')
    return
end
