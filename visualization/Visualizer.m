classdef Visualizer
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
            delete(previousStatePlot);
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
                disp(size(CO));
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
            
            fprintf("showAllMarkers? : %d\n", showAllMarkers)
            % get data
            botPositions = bot.get_history_base();
            [endPositions, plannedEndPositions] = bot.get_history_ends();
            % reduce data
            botPositions = cast(botPositions, 'single');
            endPositions = cast(endPositions, 'single');
            plannedEndPositions = cast(plannedEndPositions, 'single');
            endPositions1 = endPositions(:,1,:);
            endPositions2 = endPositions(:,2,:);
            plannedEndPositions1 = plannedEndPositions(:,1,:);
            plannedEndPositions2 = plannedEndPositions(:,2,:);
            % if walk
            if strcmp(bot.mode, 'walk')
                plannedEndPositions3 = plannedEndPositions(:,3,:);
                plannedEndPositions4 = plannedEndPositions(:,4,:);
            end

            % simulate
            b = [];
            initialBotPositions = botPositions;
            initialEndPositions1 = endPositions1;
            initialEndPositions2 = endPositions2;
            initialPlannedEndPositions1 = plannedEndPositions1;
            initialPlannedEndPositions2 = plannedEndPositions2;
            if strcmp(bot.mode, 'walk')
                initialPlannedEndPositions3 = plannedEndPositions3;
                initialPlannedEndPositions4 = plannedEndPositions4;
            end
            if showAllMarkers == false
                initialBotPositions = botPositions(1,:);
                initialEndPositions1 = endPositions1(1,:);
                initialEndPositions2 = endPositions2(1,:);
                initialPlannedEndPositions1 = plannedEndPositions1(1,:);
                initialPlannedEndPositions2 = plannedEndPositions2(1,:);
            initialPlannedEndPositions3 = plannedEndPositions3(1,:);
            initialPlannedEndPositions4 = plannedEndPositions4(1,:);
            end
            historyLineBase = animatedline(initialBotPositions(:,1),initialBotPositions(:,2), initialBotPositions(:,3),'Color','g','LineWidth',2);
            historyLineEndPosition1 = animatedline(initialEndPositions1(:,1),initialEndPositions1(:,2), initialEndPositions1(:,3),'Color','b','LineWidth',2);
            historyLineEndPosition2 = animatedline(initialEndPositions2(:,1),initialEndPositions2(:,2), initialEndPositions2(:,3),'Color','b','LineWidth',2);

            % Plot planned positions
            historyScatterPlannedEndPosition1 = animatedline(initialPlannedEndPositions1(:,1),initialPlannedEndPositions1(:,2), initialPlannedEndPositions1(:,3),'MarkerFaceColor','m', 'Marker','o','LineStyle','none');
            historyScatterPlannedEndPosition2 = animatedline(initialPlannedEndPositions2(:,1),initialPlannedEndPositions2(:,2), initialPlannedEndPositions2(:,3),'MarkerFaceColor','m', 'Marker','o','LineStyle','none');
            %Visualizer.show_text(plannedEndPositions1, startScatterI, i, '1');
            %Visualizer.show_text(plannedEndPositions2, startScatterI, i, '2');
            if strcmp(bot.mode, 'walk')
                historyScatterPlannedEndPosition3 = animatedline(initialPlannedEndPositions3(:,1),initialPlannedEndPositions3(:,2), initialPlannedEndPositions3(:,3),'MarkerFaceColor','b', 'Marker','o','LineStyle','none');
                historyScatterPlannedEndPosition4 = animatedline(initialPlannedEndPositions4(:,1),initialPlannedEndPositions4(:,2), initialPlannedEndPositions4(:,3),'MarkerFaceColor','b', 'Marker','o','LineStyle','none');
                %Visualizer.show_text(plannedEndPositions3, startScatterI, i, '3');
                %Visualizer.show_text(plannedEndPositions4, startScatterI, i, '4');
            end


            for i = beginState:stopState
                %fprintf('state #%i\n', i)
                state = states(i);
                if showAllMarkers == false
                    % history lines
                    addpoints(historyLineBase,botPositions(i,1),botPositions(i,2),botPositions(i,3));
                    addpoints(historyLineEndPosition1,endPositions1(i,1),endPositions1(i,2),endPositions1(i,3));
                    addpoints(historyLineEndPosition2,endPositions2(i,1),endPositions2(i,2),endPositions2(i,3));
    
                    % planned end positions scatter
                    addpoints(historyScatterPlannedEndPosition1,plannedEndPositions1(i,1),plannedEndPositions1(i,2),plannedEndPositions1(i,3));
                    addpoints(historyScatterPlannedEndPosition2,plannedEndPositions2(i,1),plannedEndPositions2(i,2),plannedEndPositions2(i,3));
                    if strcmp(bot.mode, 'walk')
                        addpoints(historyScatterPlannedEndPosition3,plannedEndPositions3(i,1),plannedEndPositions3(i,2),plannedEndPositions3(i,3));
                        addpoints(historyScatterPlannedEndPosition4,plannedEndPositions4(i,1),plannedEndPositions4(i,2),plannedEndPositions4(i,3));
                    end
                    drawnow
                end   

                b = Visualizer.show_joint_positions(bot, state, 'r', b); 
                pause(pauseTime);

            end
        end
    end
end
        