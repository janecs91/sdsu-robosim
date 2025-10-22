%% ===== SETUP ======
addpath('terrain');
addpath('terrain/generators');
pathDirectory = 'input_path'; 
terrainDirectory = 'input_terrain';
outputEnvDirectory = 'output_results_paper';
outputPlotDirectory = 'output_plots_sin_trends2';
outputVisualsDirectory = 'output_visuals_sin';
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
savePlots = true;
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
%% plot settings
plotLineWidth = 2;
fontSizeScale = 1.7;
fixedAmplitude = 5;
fixedFrequency = 0.01;



%% ===== SIMULATE =====
% System setup
Environment.load_paths()
addpath('input_args/input_path_args');
addpath('input_args/input_terrain_args');
addpath('input_args/input_terrain_args/sin');
executionGroup = 1;
if executionGroup == 1
    % Sin group
    pathArgInstances = {StraightPathArgs(), DiagonalPathArgs(), SinPathArgs(), HalfCirclePathArgs()};
    terrainArgInstances = {SinEasyLowTerrainArgs(), SinEasyMedTerrainArgs, SinEasyHighTerrainArgs, SinMedLowTerrainArgs, ...
        SinMedMedTerrainArgs, SinMedHighTerrainArgs, ...
        SinBumpyLowTerrainArgs, SinBumpyLowXTerrainArgs, SinBumpyLowXXTerrainArgs, ...
        SinBumpyMedTerrainArgs, SinBumpyMedXTerrainArgs, ...
        SinEasyAmp7TerrainArgs(), SinEasyAmp12TerrainArgs(), ...
        };
end

pathName = "";
terrainName = "";
figureVisibility = 'off';
if showPlots > 0
    figureVisibility = 'on';
end
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
        load(sprintf("%s/results_%s.mat", outputEnvDirectory, envName), 'env');
        
        amplitudes(terrainArgIndex) = terrainArgInstance.terrainAmplitude;
        frequencies(terrainArgIndex) = terrainArgInstance.terrainFrequency;
        times(:, terrainArgIndex) = env.totalTime(:);
        powers(:, terrainArgIndex) = env.totalPower(:);
    end
    disp(forTablePathName);
    %disp(times)
    
    %% 2d time plot
    % amplitude
    desired2DAmplitudeIndices = find(frequencies==fixedFrequency);
    %disp("Indices")
    %disp(desired2DAmplitudeIndices)
    figure('visible',figureVisibility);
    desiredAmplitudes = amplitudes(desired2DAmplitudeIndices);
    desiredTimes = times(:,desired2DAmplitudeIndices);
    [sortedAmplitudes, sortedAmplitudeIndices] = sort(desiredAmplitudes);
    plot(sortedAmplitudes, desiredTimes(:,sortedAmplitudeIndices), '-o', 'LineWidth',plotLineWidth)
    xlabel("Amplitude")
    ylabel("Time (s)")
    title(sprintf("Time Trend for Sin Terrain (fixed frequency=%.2d)", fixedFrequency))
    legend({'pull','roll','walk'},'Location','southeast')
    fontsize(gcf,scale=fontSizeScale)
    if savePlots
        pathVisualFileName = sprintf("%s/trend_sin_time2d_amp_%s.png", outputPlotDirectory, forTablePathName);
        saveas(gcf,pathVisualFileName)
    end
    % frequency
    figure('visible',figureVisibility);
    desired2DFrequencyIndices = find(amplitudes==fixedAmplitude);
    desiredFrequencies = frequencies(desired2DFrequencyIndices);
    desiredTimes = times(:,desired2DFrequencyIndices);
    [sortedFrequencies, sortedFrequencyIndices] = sort(desiredFrequencies);
    plot(sortedFrequencies, desiredTimes(:,sortedFrequencyIndices), '-o', 'LineWidth',plotLineWidth)
    xlabel("Frequency")
    ylabel("Time (s)")
    title(sprintf("Time Trend for Sin Terrain (fixed amplitude=%d)", fixedAmplitude))
    legend({'pull','roll','walk'},'Location','southeast')
    fontsize(gcf,scale=fontSizeScale)
    if savePlots
        pathVisualFileName = sprintf("%s/trend_sin_time2d_freq_%s.png", outputPlotDirectory, forTablePathName);
        saveas(gcf,pathVisualFileName)
    end
    %% 3d time plot with both amplitude & frequency
    figure('visible',figureVisibility);
    %plot3(amplitudes, frequencies, times(:,:), 'LineWidth',plotLineWidth)
    scatter3(amplitudes, frequencies, times(:,:), 'filled')
    xlabel("Amplitude")
    ylabel("Frequency")
    zlabel("Time (s)")
    title("Time Trend for Sin Terrain")
    legend({'pull','roll','walk'},'Location','northeast')
    fontsize(gcf,scale=fontSizeScale)
    if savePlots
        pathVisualFileName = sprintf("%s/trend_sin_time3d_%s.png", outputPlotDirectory, forTablePathName);
        saveas(gcf,pathVisualFileName)
    end

    %% 2d power plot
    % amplitude
    figure('visible',figureVisibility);
    desired2DAmplitudeIndices = find(frequencies==fixedFrequency);
    desiredAmplitudes = amplitudes(desired2DAmplitudeIndices);
    desiredPowers = powers(:,desired2DAmplitudeIndices);
    [sortedAmplitudes, sortedAmplitudeIndices] = sort(desiredAmplitudes);
    plot(sortedAmplitudes, desiredPowers(:,sortedAmplitudeIndices), '-o', 'LineWidth',plotLineWidth)
    xlabel("Amplitude")
    ylabel("Power (W)")
    title(sprintf("Power Trend for Sin Terrain (fixed frequency=%.2d)", fixedFrequency))
    legend({'pull','roll','walk'},'Location','southeast')
    fontsize(gcf,scale=fontSizeScale)
    if savePlots
        pathVisualFileName = sprintf("%s/trend_sin_pwr2d_amp_%s.png", outputPlotDirectory, forTablePathName);
        saveas(gcf,pathVisualFileName)
    end
    % frequency
    figure('visible',figureVisibility);
    desired2DFrequencyIndices = find(amplitudes==fixedAmplitude);
    desiredFrequencies = frequencies(desired2DFrequencyIndices);
    desiredPowers = powers(:,desired2DFrequencyIndices);
    [sortedFrequencies, sortedFrequencyIndices] = sort(desiredFrequencies);
    plot(sortedFrequencies, desiredPowers(:,sortedFrequencyIndices), '-o', 'LineWidth',plotLineWidth)
    xlabel("Frequency")
    ylabel("Power (W)")
    title(sprintf("Power Trend for Sin Terrain (fixed amplitude=%.2d)", fixedAmplitude))
    legend({'pull','roll','walk'},'Location','southeast')
    fontsize(gcf,scale=fontSizeScale)
    if savePlots
        pathVisualFileName = sprintf("%s/trend_sin_pwr2d_freq_%s.png", outputPlotDirectory, forTablePathName);
        saveas(gcf,pathVisualFileName)
    end
    %% 3d power plot with both amplitude & frequency
    figure('visible',figureVisibility);
    %plot3(amplitudes, frequencies, powers(:,:), 'LineWidth',plotLineWidth)
    scatter3(amplitudes, frequencies, powers(:,:), 'filled')
    xlabel("Amplitude")
    ylabel("Frequency")
    zlabel("Power (W)")
    title("Power Trend for Sin Terrain")
    legend({'pull','roll','walk'},'Location','northeast')
    fontsize(gcf,scale=fontSizeScale)
    if savePlots
        pathVisualFileName = sprintf("%s/trend_sin_power3d_%s.png", outputPlotDirectory, forTablePathName);
        saveas(gcf,pathVisualFileName)
    end
end
