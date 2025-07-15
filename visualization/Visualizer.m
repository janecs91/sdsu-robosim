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
            grid on;
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
        function simulate_bot(bot, pauseTime, beginState, stopState, viewAxis, showColorBar, equalAxis)
            states = bot.stateHistory;
            if nargin < 2
                pauseTime = 0.5;
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
            showAll = false;
            if beginState == length(states)
                showAll = true;
            end
            if nargin < 5
                viewAxis = "";
            end
            if nargin < 6
                showColorBar = 1;
            end
            if nargin < 7
                equalAxis = 1;
            end
            
            % change view to 3D
            view(3);
            if viewAxis ~= ""
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
            
            fprintf("showAll? : %d\n", showAll)
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
            
            % simulate
            b = [];
            startLinesI = 1;
            startScatterI = 1;
            for i = beginState:stopState
                %fprintf('state #%i\n', i)
                state = states(i);
                if showAll == false
                    startLinesI = max(i-1,1);
                    startScatterI = i;
                end
                hold on
                Visualizer.show_history_lines(botPositions, startLinesI, i, 'g');
                Visualizer.show_history_lines(endPositions1, startLinesI, i, 'b');
                Visualizer.show_history_lines(endPositions2, startLinesI, i, 'b');
                Visualizer.show_history_scatter(plannedEndPositions1, startScatterI, i, 'm');
                Visualizer.show_history_scatter(plannedEndPositions2, startScatterI, i, 'm');
                Visualizer.show_text(plannedEndPositions1, startScatterI, i, '1');
                Visualizer.show_text(plannedEndPositions2, startScatterI, i, '2');
                hold off
                b = Visualizer.show_joint_positions(bot, state, 'r', b);
                pause(pauseTime);

            end
        end
    end
end
        