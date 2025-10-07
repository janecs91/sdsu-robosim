classdef Visualizer
    properties(Constant)
        verbose = false;
    end
    properties
    end
    methods
        function obj = Visualizer()
            % load data
            %obj.terrain_path = terrain_path;
        end
    end
    methods(Static)
        function statePlot = show_joint_positions(bot, state, color, previousStatePlot)
            if nargin > 3
                delete(previousStatePlot);
            end
            %set(previousStatePlot, 'visible', 'off');
            positions = bot.get_global_joint_positions(state);
            % base
            vertices = [positions{1}(1,:); positions{2}(1,:); positions{3}(1,:); positions{4}(1,:); ...
                positions{1}(2,:); positions{2}(2,:); positions{3}(2,:); positions{4}(2,:)];
            faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 1 5 8 4; 2 6 7 3];
            basePlot = patch('Vertices',vertices,'Faces',faces,...
              'FaceVertexCData',hsv(6),'FaceColor','flat');
            %{
            text(0,0,0,['Base (' num2str(state.basePosition(1)) ',' ...
                num2str(state.basePosition(2)) ',' ...
                num2str(state.basePosition(3)) ')'])
            %}

            % use same figure
            hold on
            % legs
            legsPlot = plot3(positions{1}(:,1), positions{1}(:,2), positions{1}(:,3), ...
            positions{2}(:,1), positions{2}(:,2), positions{2}(:,3), ...
            positions{3}(:,1), positions{3}(:,2), positions{3}(:,3), ...
            positions{4}(:,1), positions{4}(:,2), positions{4}(:,3), 'color', color, ...
            'LineWidth', 2);
            textHandles = [];
            jointsAllText = "";
            for i=1:4
                t2 = text(positions{i}(1,1), positions{i}(1,2), positions{i}(1,3), sprintf('L%i',i));
                set(t2,'Rotation',90);
                jointsLegText = sprintf('L%i J(%.1f,%.1f,%.1f)\n',i, ...
                    rad2deg(state.anglesWaist(i)), rad2deg(state.anglesHip(i)), rad2deg(state.anglesKnee(i)));
                jointsAllText = strcat(jointsAllText, jointsLegText);
                %set(t3,'BackgroundColor','white');
                textHandles = [textHandles; t2];
            end
            t3 = text(0,0,-4, jointsAllText, ...
                    'FontSize', 10);
            textHandles = [textHandles; t3];
            hold off
            
            %{
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            %}
            %grid on;
            statePlot = [basePlot; legsPlot; textHandles];
        end
        function botHistoryPlot = show_history_scatter(positions, startI, endI, color)
            if nargin < 3
                color = 'b';
            end
            botHistoryPlot = scatter3(positions(startI:endI,1), positions(startI:endI,2), ...
                positions(startI:endI,3), 'MarkerFaceColor',color);
        end
        function textHandles = show_text(positions, startI, endI, stringOutput)
            textHandles = [];
            t2 = text(double(positions(startI:endI,1)), double(positions(startI:endI,2)), ...
                double(positions(startI:endI,3)), stringOutput);
            %set(t2,'Rotation',90);
            textHandles = [textHandles; t2];
        end
        function botHistoryPlot = show_history_lines(positions, startI, endI, color)
            if nargin < 3
                color = 'b';
            end
            botHistoryPlot = plot3(positions(startI:endI, 1), positions(startI:endI, 2), ...
                positions(startI:endI, 3), 'Color', color, 'LineWidth', 2);
        end
        function terrainPlot = simulate_terrain(terrain)
            % use same figure if any
            % check terrain type
            if isa(terrain, 'MatrixTerrain')
                % matrix terrain
                %terrainX = terrain.initX:terrain.scaleX:terrain.maxX;
                %terrainY = terrain.initY:terrain.scaleY:terrain.maxY;
                terrainSize = terrain.gridSize;
                CO(:,:,1) = zeros(terrainSize(2)); % red
                if Visualizer.verbose
                    fprintf("Terrain size %d %d", size(CO));
                end
                %CO(:,:,2) = ones(terrainSize(2)).*linspace(0.5,0.6,25); % green
                %CO(:,:,3) = ones(terrainSize(2)).*linspace(0,1,25); % blue
                terrainPlot = surf(terrain.initX:terrain.scaleX:terrain.maxX, ...
                terrain.initY:terrain.scaleY:terrain.maxY, ...
                terrain.elevationMatrix', 'FaceAlpha',0.5);
            else   
                % equation terrain
                k1 = 1; %how many times you want wave to oscillate in x-dir
                k2 = 0; %how many times you want wave to oscillate in y-dir
                minXY = -pi;
                maxXY = pi;
                [X,Y] = meshgrid(linspace(minXY, maxXY));
                Z = cos(k1*X).*cos(k2*Y);
                terrainPlot = surf(X,Y,Z);
            end
        end
        function pathPlot = simulate_path(path)
            pathPlot = plot(path.pathPoints(:,1), path.pathPoints(:,2), 'Color', 'y', ...
                'LineWidth', 2);
        end
        % pauseTime = seconds to wait until next frame
        % stopState = max # of states to show
        function simulate_bot(bot, pauseTime, beginState, stopState, showAllMarkers, viewAxis, showColorBar, equalAxis)
            grid on;
            states = bot.stateHistory;
            if nargin < 2
                pauseTime = 0.001;
            end
            if nargin < 3
                beginState = 1;
            else
                beginState = min(max(beginState,1), length(states));
            end
            if nargin < 4 || stopState == -1
                stopState = length(states);
            else
                stopState = min(max(stopState,1), length(states));
            end
            if nargin < 5
                showAllMarkers = false;
            end
            if beginState == length(states)
                showAllMarkers = true;
            end
            if nargin < 6
                viewAxis = "";
            end
            if nargin < 7
                showColorBar = 1;
            end
            if nargin < 8
                equalAxis = 1;
            end
            
            % change view to 3D
            view(3);
            if ~strcmp(viewAxis, '')
                if strcmp(viewAxis, 'x')
                    view(90,0);
                elseif strcmp(viewAxis, 'y')
                    view(0,0);
                    disp('axis')
                    disp(axis)
                    axis([0 inf -inf inf -inf inf]);
                elseif strcmp(viewAxis, 'z')
                    view(0, 90);
                end
            end
            if equalAxis == 1
                axis equal;
            end
            if showColorBar > 0
                colorbar;
            end
            %zoom off;

            isNotRoll = ~strcmp(bot.mode, 'roll');
            isWalk = strcmp(bot.mode, 'walk');
            if Visualizer.verbose
                fprintf("isnotRoll? %d\n", isNotRoll);
                fprintf("isWalk? %d\n", isWalk);
                
                fprintf("showAllMarkers? : %d\n", showAllMarkers)
            end
            % get data
            botPositions = bot.get_history_base();
            
            % reduce data
            [endPositions, plannedEndPositions] = bot.get_history_ends();
            botPositions = cast(botPositions, 'single');
            endPositions = cast(endPositions, 'single');
            endPositions1 = endPositions(:,1,:);
            endPositions2 = endPositions(:,2,:);
            if isNotRoll
                plannedEndPositions = cast(plannedEndPositions, 'single');
                plannedEndPositions1 = plannedEndPositions(:,1,:);
                plannedEndPositions2 = plannedEndPositions(:,2,:);
            else
                plannedEndPositions = [];
            end
            
            % if walk
            if isWalk
                plannedEndPositions3 = plannedEndPositions(:,3,:);
                plannedEndPositions4 = plannedEndPositions(:,4,:);
            end

            % simulate
            b = [];
            initialBotPositions = botPositions;
            initialEndPositions1 = endPositions1;
            initialEndPositions2 = endPositions2;
            if isNotRoll
                initialPlannedEndPositions1 = plannedEndPositions1;
                initialPlannedEndPositions2 = plannedEndPositions2;
            end
            if isWalk
                initialPlannedEndPositions3 = plannedEndPositions3;
                initialPlannedEndPositions4 = plannedEndPositions4;
            end
            if showAllMarkers == false
                initialBotPositions = botPositions(1,:);
                initialEndPositions1 = endPositions1(1,:);
                initialEndPositions2 = endPositions2(1,:);
                if isNotRoll
                    initialPlannedEndPositions1 = plannedEndPositions1(1,:);
                    initialPlannedEndPositions2 = plannedEndPositions2(1,:);
                end
                if isWalk
                    initialPlannedEndPositions3 = plannedEndPositions3(1,:);
                    initialPlannedEndPositions4 = plannedEndPositions4(1,:);
                end
            end
            historyLineBase = animatedline(initialBotPositions(:,1),initialBotPositions(:,2), initialBotPositions(:,3),'Color','g','LineWidth',2);
            historyLineEndPosition1 = animatedline(initialEndPositions1(:,1),initialEndPositions1(:,2), initialEndPositions1(:,3),'Color','b','LineWidth',2);
            historyLineEndPosition2 = animatedline(initialEndPositions2(:,1),initialEndPositions2(:,2), initialEndPositions2(:,3),'Color','b','LineWidth',2);

            % Plot planned positions
            if isNotRoll
                historyScatterPlannedEndPosition1 = animatedline(initialPlannedEndPositions1(:,1),initialPlannedEndPositions1(:,2), initialPlannedEndPositions1(:,3),'MarkerFaceColor','m', 'Marker','o','LineStyle','none');
                historyScatterPlannedEndPosition2 = animatedline(initialPlannedEndPositions2(:,1),initialPlannedEndPositions2(:,2), initialPlannedEndPositions2(:,3),'MarkerFaceColor','m', 'Marker','o','LineStyle','none');
                %Visualizer.show_text(plannedEndPositions1, startScatterI, i, '1');
                %Visualizer.show_text(plannedEndPositions2, startScatterI, i, '2');
            end
            if isWalk
                historyScatterPlannedEndPosition3 = animatedline(initialPlannedEndPositions3(:,1),initialPlannedEndPositions3(:,2), initialPlannedEndPositions3(:,3),'MarkerFaceColor','black', 'Marker','o','LineStyle','none');
                historyScatterPlannedEndPosition4 = animatedline(initialPlannedEndPositions4(:,1),initialPlannedEndPositions4(:,2), initialPlannedEndPositions4(:,3),'MarkerFaceColor','black', 'Marker','o','LineStyle','none');
                %Visualizer.show_text(plannedEndPositions3, startScatterI, i, '3');
                %Visualizer.show_text(plannedEndPositions4, startScatterI, i, '4');
            end


            for i = 1:stopState
                state = states(i);
                if Visualizer.verbose
                    fprintf('state #%i\n', i)
                    fprintf('state base position: %d %d %d\n', state.basePosition(:))
                end
                if showAllMarkers == false
                    % history lines
                    addpoints(historyLineBase,botPositions(i,1),botPositions(i,2),botPositions(i,3));
                    addpoints(historyLineEndPosition1,endPositions1(i,1),endPositions1(i,2),endPositions1(i,3));
                    addpoints(historyLineEndPosition2,endPositions2(i,1),endPositions2(i,2),endPositions2(i,3));
    
                    % planned end positions scatter
                    if isNotRoll
                        addpoints(historyScatterPlannedEndPosition1,plannedEndPositions1(i,1),plannedEndPositions1(i,2),plannedEndPositions1(i,3));
                        addpoints(historyScatterPlannedEndPosition2,plannedEndPositions2(i,1),plannedEndPositions2(i,2),plannedEndPositions2(i,3));
                    end
                    if isWalk
                        addpoints(historyScatterPlannedEndPosition3,plannedEndPositions3(i,1),plannedEndPositions3(i,2),plannedEndPositions3(i,3));
                        addpoints(historyScatterPlannedEndPosition4,plannedEndPositions4(i,1),plannedEndPositions4(i,2),plannedEndPositions4(i,3));
                    end
                    drawnow
                end   

                if i >= beginState
                    b = Visualizer.show_joint_positions(bot, state, 'r', b); 
                    pause(pauseTime);
                end

            end
        end

        function simulate_bot_single_frame(bot, stateNumberPercent, equalAxis, showAllMarkers, viewAxis, showColorBar)
            % stateNumberPercent is a decimal from 0 (0%) to 1 (100%)
            grid on;
            states = bot.stateHistory;
            stateNumber = ceil(stateNumberPercent*length(states));
            if nargin < 3
                showAllMarkers = false;
            end
            if stateNumber == length(states)
                showAllMarkers = true;
            end
            if nargin < 4
                viewAxis = "";
            end
            if nargin < 5
                showColorBar = 1;
            end
            if nargin < 6
                equalAxis = 1;
            end
            
            % change view to 3D
            view(3);
            if ~strcmp(viewAxis, '')
                if strcmp(viewAxis, 'x')
                    view(90,0);
                elseif strcmp(viewAxis, 'y')
                    view(0,0);
                    if Visualizer.verbose
                        disp('axis')
                        disp(axis)
                    end
                    axis([0 inf -inf inf -inf inf]);
                elseif strcmp(viewAxis, 'z')
                    view(0, 90);
                end
            end
            if equalAxis == 1
                axis equal;
            end
            if showColorBar > 0
                colorbar;
            end
            %zoom off;

            isNotRoll = ~strcmp(bot.mode, 'roll');
            isWalk = strcmp(bot.mode, 'walk');
            if Visualizer.verbose
                fprintf("isnotRoll? %d\n", isNotRoll);
                fprintf("isWalk? %d\n", isWalk);
                
                fprintf("showAllMarkers? : %d\n", showAllMarkers)
            end
            % get data
            botPositions = bot.get_history_base();
            
            % reduce data
            [endPositions, plannedEndPositions] = bot.get_history_ends();
            botPositions = cast(botPositions, 'single');
            endPositions = cast(endPositions, 'single');
            endPositions1 = endPositions(:,1,:);
            endPositions2 = endPositions(:,2,:);
            if isNotRoll
                plannedEndPositions = cast(plannedEndPositions, 'single');
                plannedEndPositions1 = plannedEndPositions(:,1,:);
                plannedEndPositions2 = plannedEndPositions(:,2,:);
            else
                plannedEndPositions = [];
            end
            
            % if walk
            if isWalk
                plannedEndPositions3 = plannedEndPositions(:,3,:);
                plannedEndPositions4 = plannedEndPositions(:,4,:);
            end

            % simulate
            b = [];
            initialBotPositions = botPositions;
            initialEndPositions1 = endPositions1;
            initialEndPositions2 = endPositions2;
            if isNotRoll
                initialPlannedEndPositions1 = plannedEndPositions1;
                initialPlannedEndPositions2 = plannedEndPositions2;
            end
            if isWalk
                initialPlannedEndPositions3 = plannedEndPositions3;
                initialPlannedEndPositions4 = plannedEndPositions4;
            end
            if showAllMarkers == false
                initialBotPositions = botPositions(1:stateNumber,:);
                initialEndPositions1 = endPositions1(1:stateNumber,:);
                initialEndPositions2 = endPositions2(1:stateNumber,:);
                if isNotRoll
                    initialPlannedEndPositions1 = plannedEndPositions1(1,:);
                    initialPlannedEndPositions2 = plannedEndPositions2(1,:);
                end
                if isWalk
                    initialPlannedEndPositions3 = plannedEndPositions3(1,:);
                    initialPlannedEndPositions4 = plannedEndPositions4(1,:);
                end
            end
            historyLineBase = plot3(initialBotPositions(:,1),initialBotPositions(:,2), initialBotPositions(:,3),'Color','g','LineWidth',2);
            historyLineEndPosition1 = plot3(initialEndPositions1(:,1),initialEndPositions1(:,2), initialEndPositions1(:,3),'Color','b','LineWidth',2);
            historyLineEndPosition2 = plot3(initialEndPositions2(:,1),initialEndPositions2(:,2), initialEndPositions2(:,3),'Color','b','LineWidth',2);

            % Plot planned positions
            if isNotRoll
                historyScatterPlannedEndPosition1 = scatter3(initialPlannedEndPositions1(:,1),initialPlannedEndPositions1(:,2), initialPlannedEndPositions1(:,3),'MarkerFaceColor','m', 'Marker','o');
                historyScatterPlannedEndPosition2 = scatter3(initialPlannedEndPositions2(:,1),initialPlannedEndPositions2(:,2), initialPlannedEndPositions2(:,3),'MarkerFaceColor','m', 'Marker','o');
                %Visualizer.show_text(plannedEndPositions1, startScatterI, i, '1');
                %Visualizer.show_text(plannedEndPositions2, startScatterI, i, '2');
            end
            if isWalk
                historyScatterPlannedEndPosition3 = scatter3(initialPlannedEndPositions3(:,1),initialPlannedEndPositions3(:,2), initialPlannedEndPositions3(:,3),'MarkerFaceColor','black', 'Marker','o');
                historyScatterPlannedEndPosition4 = scatter3(initialPlannedEndPositions4(:,1),initialPlannedEndPositions4(:,2), initialPlannedEndPositions4(:,3),'MarkerFaceColor','black', 'Marker','o');
                %Visualizer.show_text(plannedEndPositions3, startScatterI, i, '3');
                %Visualizer.show_text(plannedEndPositions4, startScatterI, i, '4');
            end

            % draw state
            state = states(stateNumber);
            if Visualizer.verbose
                fprintf('state #%i\n', stateNumber)
                fprintf('state base position: %d %d %d\n', state.basePosition(:))
            end
              
            b = Visualizer.show_joint_positions(bot, state, 'r'); 

        end
        
    end
end
        