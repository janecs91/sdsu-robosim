classdef BaseBot
    properties
        debug;
        % bot values
        botHeight;
        botBodyLength = 30;
        maxBotHeight = 90;  % check?
        %maxLegLength = 48;
        %{
a0 = d0 = 15.5cm = half width of robot body
d1 = 15cm = half height of robot body
a2 = 6cm = distance from robot body to first joint
a3 = a4 = 24 cm = leg length
        %}
        
        % saved functions
        functionEndPositions;
        functionsRotLeg;
        functionAByWheels;
        functionsAByLeg;
        functionsAByBody;
        functionJointPositions;
        
        % state history
        stateHistory;
        previousState;
        
        % data matrix
        dataMatrix;
        baseMatrix;
        endPositionsMatrix;
        jointsMatrix;
        jointChangesMatrix;
        dotJointsMatrix;
        dotEndPositionsMatrix;
        plannedEndPositionsMatrix;
        condMatrix;
        
        % path
        pathAccuracyRadius = 24*sqrt(2);
        
        % constants
        constants;
        a_0;
        a_2;
        a_3;
        a_4;
        d_1;
        kinematics;
        
        % inital joint angles
        % waist range: -45 to 45 (side-front)
        % hip range: 0 (straight up) 90 (flat) to 120 (orig) 180 (down)
        % knee: 0 (folded) 60 (orig) 180(stretched)
        % orig paper knee: [60 60 0 0]
        waist = [deg2rad(45) deg2rad(-45) 0 0];
        hip = [deg2rad(90) deg2rad(90) deg2rad(120) deg2rad(120)];
        knee = [deg2rad(90) deg2rad(90) deg2rad(0) deg2rad(0)];
        steering = [0 0 -3*pi/4 3*pi/4];
        contact = [0 0 0 0];
        
        % analysis
        costAnalyzer;
    end
    properties(Abstract)
        % mode: roll, walk, pull
        mode;
        maxStepSize;
    end
    properties(Constant)
        % flags
        % saveAllStatesFlag: keep all state objects in history (not necessary)
        % collected data is still preserved as matrix of scalar values
        saveAllStatesFlag = true;
        % pinvAtRuntimeFlag = does pinv() on matrix A at runtime
        pinvAtRuntimeFlag = false;
    end
    methods
        function obj = BaseBot()
            % load data
            obj.botHeight = obj.get_init_bot_height();
            addpath('transformations');
            obj.constants = Transformation.constants;
            obj.a_0 = obj.constants('a_0');
            obj.a_2 = obj.constants('a_2');
            obj.a_4 = obj.constants('a_4');
            obj.a_3 = obj.constants('a_3');
            obj.d_1 = obj.constants('d_1');
            
            obj.debug = 0;
            
            addpath('robot/kinematics');
            addpath('robot/analyzer');
            %obj.costAnalyzer = CostAnalyzer();
        end
        function state = get_first_state(obj)
            state = obj.stateHistory(1);
        end
        function state = get_last_state(obj)
            state = obj.previousState;
        end
        function obj = add_history(obj, newState)
            obj.previousState = newState;
            if obj.saveAllStatesFlag == true
                obj.stateHistory = [obj.stateHistory; newState];
            end
            obj.dataMatrix = [obj.dataMatrix; newState.to_vector()];
            obj.baseMatrix = [obj.baseMatrix; newState.to_vector_base()];
            obj.jointsMatrix = [obj.jointsMatrix; newState.to_vector_joints()];
            obj.jointChangesMatrix = [obj.jointChangesMatrix; newState.to_vector_dot_joints()];
            obj.endPositionsMatrix = cat(3, obj.endPositionsMatrix, newState.to_vector_end_positions());
            obj.plannedEndPositionsMatrix = cat(3, obj.plannedEndPositionsMatrix, newState.to_vector_planned_end_positions_flatten());
        end
        function obj = clear_history(obj)
            obj.stateHistory = [];
            obj.dataMatrix = [];
            obj.baseMatrix = [];
            obj.jointsMatrix = [];
            obj.jointChangesMatrix = [];
            obj.endPositionsMatrix = [];
            obj.plannedEndPositionsMatrix = [];
        end
        function matrix = history_to_matrix(obj)
            matrix = [];
            for i = 1:length(obj.stateHistory)
                state = obj.stateHistory(i);
                matrix = [matrix; state.to_vector()];
            end
        end
        function state = update_orientation(obj, newOrientation)
            % ???? do I need this?
            state = obj.get_last_state();
            state.baseOrientation = newOrientation;
        end
        function obj = update_initial_state_in_history(obj, initialState)
            obj.stateHistory(1) = initialState;
        end
        
        %% Initialize
        function obj = initialize(obj, terrain, path, startX)
            disp("BASE BOT INITALIZE METHOD ************");
            % TBD: startX (replaces initialPosition)
            %initialPosition = obj.get_init_position(terrain, path);
            %initState = obj.init_state(initialPosition, terrain);
            initState = obj.init_state(terrain, path, startX);
            obj = obj.add_history(initState);
        end
        function initPosition = get_init_position(obj, terrain, path)
            %initPosition(1) = terrain.initX + obj.botBodyLength;
            %initPosition(2) = (terrain.maxY+terrain.initY)/2;
            initPosition(1:2) = path.get_first_point();
            initPosition(3) = terrain.get_elevation(initPosition(1), initPosition(2));
        end
        function state = init_state(obj, terrain, path, initPosition, initGamma, waist, hip, knee, steering, contact)
            usePathGamma = true;
            if nargin < 4
                initPosition = obj.get_init_position(terrain, path);
            end
            if nargin > 5
                usePathGamma = false;
            end
            if nargin < 6
                waist = obj.waist;
            end
            if nargin < 7
                hip = obj.hip;
            end
            if nargin < 8
                knee = obj.knee;
            end
            if nargin < 9
                steering = obj.steering;
            end
            if nargin < 10
                contact = obj.contact;
            end
            if initPosition == -1
                initPosition = obj.get_init_position(terrain, path);
            end
            if isscalar(initPosition) || (initPosition(1) ~= path.pathPoints(1,1) && usePathGamma == true)
                [y, pathGamma] = path.get_y_and_gamma_at_x(initPosition(1));
            else
                [pathGamma, slope] = path.get_gamma_at_index(1);
            end
            if isscalar(initPosition)
                initPosition(2) = y;
                initPosition(3) = terrain.get_elevation(initPosition(1), initPosition(2));
            end
            if usePathGamma == true
                initGamma = pathGamma;
            end
            
            state = BotState();
            state.basePosition = initPosition;
            state.baseOrientation = [0 0 initGamma];
            state.dotBaseOrientation = [0 0 0];
            state.dotEndOrientations = zeros(4,3);
            % waist range: -45 to 45 (side-front)
            state.anglesWaist = waist;
            %state.anglesWaist = [deg2rad(25) deg2rad(-25) 0 0];
            % hip range: 0 (straight up) 90 (flat) to 120 (orig) 180 (down)
            state.anglesHip = hip;
            % knee: 0 (folded) 60 (orig) 180(stretched)
            % orig paper knee: [60 60 0 0]
            state.anglesKnee = knee;
            %state.anglesKnee = [0 0 0 0];
            state.anglesSteering = steering;
            state.anglesContact = contact;
            % fix init Z
            %obj.botHeight = obj.get_bot_height(state)+10;
            obj.botHeight = obj.get_bot_height(state);
            state.basePosition(3) = obj.botHeight + obj.get_lowest_point(state, terrain);
            %state.basePosition(3) = 39;
            state.endPositions = obj.get_global_end_positions(state);
            % reset dot values
            state.clear_dots();
        end
        
        %% Traverse
        function obj = traverse_terrain(obj, terrain, path, startX, maxI)
            if nargin < 4
                startX = -1;
            end
            obj = obj.initialize(terrain, path, startX);
            %{
            lastState = obj.get_last_state();
            initPosition = lastState.basePosition;
            if nargin < 4
                startX = initPosition(1);
                startY = initPosition(2);
            else
                if startX == -1
                    startX = initPosition(1);
                end
                [startY, pathGamma] = path.get_y_and_gamma_at_x(startX);
            end
            %}
            if nargin < 5
                maxI = obj.maxI;
            end
            %maxX = (terrain.maxX - terrain.initX);
            %maxX = maxX - obj.botBodyLength;
            maxCenterX = terrain.maxX - (2*obj.botBodyLength);
            endX = path.pathPoints(end,1)-48;
            x = startX;
            i = 0;
            %moveVector = [obj.maxStepSize 0];
            %yaw = atan2(moveVector(1)*turn,moveVector(1));
            while x < endX && x < maxCenterX && i < maxI
                lastState = obj.get_last_state();
                %{
                obj.currentPathIndex = obj.nextPathIndex;
                [futurePoint, futurePathIndex] = obj.get_next_path_point(lastState, path);
                obj.nextPathIndex = futurePathIndex;
                %}
                %[turnAngle, moveVector] = obj.get_body_vector(lastState, futurePoint);
                %fprintf('angle %.2f, botyaw %.2f turnAngle %.2f\n', rad2deg(angle), rad2deg(lastState.baseOrientation(3)), rad2deg(turnAngle));
                moveVector = [obj.maxStepSize 0];
                turnAngle = 0;
                obj = obj.move_next_step(moveVector, turnAngle, terrain, path);
                x = lastState.basePosition(1);
                if lastState.pathIndex == size(path.pathPoints,1)
                    break
                end
                i = i + 1;
                fprintf("x %.2f endX %.2f maxCenterX %.2f x<endX? %d x<maxCenterX %d \n", x, endX, maxCenterX, x<endX, x<maxCenterX)
            end
        end
        function [distance, angle] = get_vector_to_point(obj, point1, point2)
            %x1 = state.basePosition(1);
            %y1 = state.basePosition(2);
            x1 = point1(1);
            y1 = point1(2);
            x2 = point2(1);
            y2 = point2(2);
            distance = sqrt((x1-x2).^2+(y1-y2).^2);
            angle = atan2(y2-y1, x2-x1);
            %fprintf("vANGLE %.2f DIST %.2f\n", rad2deg(angle), distance)
            %angle = angle + (angle < 0)*2*pi;
        end
        function [futurePoint, futurePathIndex] = get_next_path_point(obj, lastState, path, radius)
            if nargin < 4
                radius = obj.pathAccuracyRadius;
                disp("using default radius")
            end
            lastPosition = lastState.basePosition;
            [futurePoint, futurePathIndex] = path.get_next_nearest_point(lastPosition(1), lastPosition(2), ...
                lastState.pathIndex, radius);
        end
        function [turnAngle, moveVector] = get_body_vector(obj, lastState, futurePoint)
            [distance, angle] = obj.get_vector_to_point(lastState.basePosition(1:2), futurePoint);
            vectorToPathPoint = [distance, angle];
            turnAngle = angle - lastState.baseOrientation(3);
            moveVector = [min(distance, obj.maxStepSize) 0];
        end
        
        %% Helper Methods
        
        function lowestElevation = get_lowest_point(obj, state, terrain)
            baseElevation = terrain.get_elevation(state.basePosition(1), state.basePosition(2));
            feetElevations = terrain.get_elevations(state.endPositions(:,1), state.endPositions(:,2));
            lowestElevation = min([baseElevation; feetElevations(:)]);
        end
        function basePositions = get_history_base(obj)
            numStates = size(obj.stateHistory,1);
            basePositions = zeros(numStates,3);
            for i=1:numStates
                state = obj.stateHistory(i);
                basePositions(i,:) = state.basePosition;
            end
        end
        function [endPositions, plannedEndPositions] = get_history_ends(obj)
            numStates = size(obj.stateHistory,1);
            endPositions = zeros(numStates,4,3);
            plannedEndPositions = zeros(numStates,4,3);
            for i=1:numStates
                state = obj.stateHistory(i);
                endPositions(i,:,:) = state.endPositions;
                plannedEndPositions(i,:,:) = state.plannedEndPositions;
            end
        end
        function obj = update_bot_height(obj)
            state = obj.get_last_state();
            obj.botHeight = obj.get_bot_height(state);
        end
        function botHeight = get_bot_height(obj, state)
            % fix init Z
            % get_global_joint_positions(obj, state)
            %botHeight = state.basePosition(3) - min(state.endPositions(:,3));
            positions = obj.get_global_joint_positions(state);
            baseZs = zeros(1,4);
            endZs = zeros(1,4);
            for i=1:length(positions)
                positions_i = positions{i};
                baseZs(i) = positions_i(1, 3);
                endZs(i) = positions_i(size(positions_i, 1), 3);
            end
            botHeight = max(baseZs) - min(endZs);
            
        end
        function angle = get_expected_alpha(obj, state)
            % roll: compare left/right sides of robot and find angle
            state.endPositions = obj.get_global_end_positions(state);
            left = [2 3];
            right = [1 4];
            x1 = state.endPositions(left,1)+state.dotEndPositions(left,1);
            y1 = state.endPositions(left,2)+state.dotEndPositions(left,2);
            z1 = state.endPositions(left,3)+state.dotEndPositions(left,3);
            x2 = state.endPositions(right,1)+state.dotEndPositions(right,1);
            y2 = state.endPositions(right,2)+state.dotEndPositions(right,2);
            z2 = state.endPositions(right,3)+state.dotEndPositions(right,3);
            distances = sqrt((x1-x2).^2+(y1-y2).^2);
            angles = atan2(z2-z1, distances);
            angle = mean(angles);
        end
        function angle = get_expected_beta(obj, state)
            % roll bot
            % pitch: compare front/back sides of robot and find angle
            state.endPositions = obj.get_global_end_positions(state);
            front = [1 2];
            back = [3 4];
            x1 = state.endPositions(front,1)+state.dotEndPositions(front,1);
            y1 = state.endPositions(front,2)+state.dotEndPositions(front,2);
            z1 = state.endPositions(front,3)+state.dotEndPositions(front,3);
            x2 = state.endPositions(back,1)+state.dotEndPositions(back,1);
            y2 = state.endPositions(back,2)+state.dotEndPositions(back,2);
            z2 = state.endPositions(back,3)+state.dotEndPositions(back,3);
            distances = sqrt((x1-x2).^2+(y1-y2).^2);
            angles = atan2(z2-z1, distances);
            angle = mean(angles);
        end
        function rotatedVector = rotate_vector(obj, vector, theta)
            rotatedVector = [cos(theta) -sin(theta); sin(theta)  cos(theta)]*vector';
            rotatedVector = rotatedVector';
        end
        % 4x3 matrix
        function positions = get_global_end_positions(obj, state, leg)
            % calc x,y,z end positions for all legs
            % get base position + rotationMatrix*[x,y,z]_leg
            positions = obj.functionEndPositions(state.anglesWaist, state.anglesHip, state.anglesKnee, ...
                state.basePosition, state.baseOrientation);
            if nargin > 2
                positions = positions(leg,:);
            end
        end
        function localPositions = get_local_end_positions(obj, state, leg)
            % calc x,y,z end positions for all legs
            % get base position + rotationMatrix*[x,y,z]_leg
            globalPositions = obj.functionEndPositions(state.anglesWaist, state.anglesHip, state.anglesKnee, ...
                state.basePosition, state.baseOrientation);
            basePositions = repmat(state.basePosition,[size(globalPositions,1) 1]);
            baseGamma = state.baseOrientation(3);
            localPositions = globalPositions - basePositions;
            if baseGamma ~= 0
                for i=1:size(localPositions,1)
                    localPositions(i,1:2) = obj.rotate_vector(localPositions(i,1:2), -baseGamma);
                end
            end
            if nargin > 2
                localPositions = localPositions(leg,:);
            end
        end
        function state = update_end_positions(obj, state)
            state.endPositions = obj.get_global_end_positions(state);
        end
        function positions = get_global_joint_positions(obj, state, leg)
            % account for base orientation
            [leg1, leg2, leg3, leg4] = obj.functionJointPositions(state.anglesWaist, state.anglesHip, state.anglesKnee, state.anglesSteering, state.basePosition, state.baseOrientation);
            positions = {leg1, leg2, leg3, leg4};
            if nargin > 2
                positions = positions{leg};
            end 
        end
        function localPositions2 = get_local_joint_positions(obj, state, leg)
            if nargin>2
                globalPositions = obj.get_global_joint_positions(state, leg);
                globalPositions = {globalPositions};
            else
                globalPositions = obj.get_global_joint_positions(state);
            end
            %disp('global Pos');
            %disp(globalPositions{1});
            localPositions = cell(size(globalPositions,2),1);
            localPositions2 = cell(size(globalPositions,2),1);
            baseGamma = state.baseOrientation(3);
            for i=1:size(globalPositions,2)
                basePositions = repmat(state.basePosition,[size(globalPositions{i},1) 1]);
                localPositionsValues = globalPositions{i}-basePositions;
                globalPosition = globalPositions{i};
                localPositionsValues2 = zeros(size(globalPosition));
                %localPositionsValues(:,3) = globalPositions{i}(:,3);
                if baseGamma ~= 0
                    for j=1:size(localPositionsValues,1)
                        localPositionsValues(j,1:2) = obj.rotate_vector(localPositionsValues(j,1:2), -baseGamma);
                    end
                end
                for k=1:size(localPositionsValues,1)
                    localPositionsValues2(k,:) = obj.change_global_to_local(state, globalPosition(k,:));
                end
                localPositions{i} = localPositionsValues;
                localPositions2{i} = localPositionsValues2;
            end
            %{
            if nargin > 2
                localPositions = localPositions{1};
            end
            %}
            %{
            disp("global")
            disp(globalPositions{:})
            disp("local pos 1")
            disp(localPositions{:})
            disp("local pos 2")
            disp(localPositions2{:})
            %}
        end
        function localZ = change_global_z_to_local_z(obj, state, globalZ)
            localZ = globalZ-state.basePosition(3);
        end
        function localPoint = change_global_to_local(obj, state, globalPoint)
            %{
            x_a_prime = point1(1);
            y_a_prime = point1(2);
            baseX = state.basePosition(1);
            baseY = state.basePosition(2);
            gamma_prime = state.baseOrientation(3);
            invRotMatrix = [cos(gamma_prime) sin(gamma_prime) ...
                -cos(gamma_prime)*baseX-sin(gamma_prime)*baseY; ...
                -sin(gamma_prime) cos(gamma_prime) sin(gamma_prime)*baseX-cos(gamma_prime)*baseY; ...
                0 0 1];
            transVector = [x_a_prime y_a_prime 1]';
            localVector = invRotMatrix*transVector;
            localVector = localVector';
            %}
            x_a_prime = globalPoint(1);
            y_a_prime = globalPoint(2);
            baseX = state.basePosition(1);
            baseY = state.basePosition(2);
            gamma_prime = state.baseOrientation(3);
            invRotMatrix = [cos(gamma_prime) sin(gamma_prime) ...
                -cos(gamma_prime)*baseX-sin(gamma_prime)*baseY; ...
                -sin(gamma_prime) cos(gamma_prime) sin(gamma_prime)*baseX-cos(gamma_prime)*baseY; ...
                0 0 1];
            transVector = [x_a_prime y_a_prime 1]';
            localPoint = invRotMatrix*transVector;
            localPoint = localPoint';
            if size(globalPoint,2) > 2
                localPoint(3) = globalPoint(3)-state.basePosition(3);
            end
        end
        function waistZ = get_waist_z(obj, state)
            globalPositions = obj.get_global_joint_positions(state,1);
            waistZ = globalPositions(2,3);
        end
        function get_distance_hip_to_foot(obj, state, leg)
        end
        function distance = get_distance_waist_to_foot(obj, state, leg)
            globalPositions = obj.get_global_joint_positions(state, leg);
            [distance, angle] = obj.get_vector_to_point(globalPositions(2,1:2), globalPositions(end,1:2));
        end
        function distance = get_distance_base_to_foot(obj, state, leg)
            jointPositions = obj.get_local_joint_positions(state, leg);
            jointPositions = jointPositions{1};
            [distance, angle] = obj.get_vector_to_point([0 0], jointPositions(end,1:2));
        end
        function distance = get_max_height_from_waist_to_foot(obj, state)
            waistZ = obj.get_waist_z(state);
            lowestFootZ = min(state.endPositions(:,3));
            distance = waistZ - lowestFootZ;
        end
        function newState = get_new_state_given_vector_and_gamma(obj, state, dx_b, dy_b, gamma, update_end_positions_bool)
            if nargin < 6
                update_end_positions_bool = false;
            end
            globalMoveVector = obj.rotate_vector([dx_b dy_b], state.baseOrientation(3));
            newState = copy(state);
            newState.basePosition(1:2) = newState.basePosition(1:2)+globalMoveVector;
            newState.baseOrientation(3) = newState.baseOrientation(3)+gamma;
            % also get new end positions?
            if update_end_positions_bool == true
                newState = obj.update_end_positions(newState);
            end
        end
        function [x_a_prime, y_a_prime] = get_new_location(obj, x_a, y_a, dx_b, dy_b, gamma)
            newLocalPosition = [(cos(gamma)*x_a+sin(gamma)*y_a+dx_b) (-sin(gamma)*x_a+cos(gamma)*y_a+dy_b)];
            x_a_prime = newLocalPosition(1);
            y_a_prime = newLocalPosition(2);
        end
        
        
        %% Cost Analysis
        function [totalTimeCost, totalJointChanges, timeCosts, velocity, distance] = analyze_time(obj, terrain)
            [totalTimeCost, totalJointChanges, timeCosts, velocity, distance] = obj.costAnalyzer.analyze_time(obj, terrain);
        end
        function maxPowerCost = analyze_power(obj, terrain)
            maxPowerCost = obj.costAnalyzer.analyze_power(obj, terrain);
        end
    end
    methods(Static)
        function botHeight = get_init_bot_height()
            % logic from silo       
            addpath('transformations/');
            botHeight = Transformation.constants('d_1') ...
                + Transformation.constants('a_4') ...
                + Transformation.constants('d_5') ...
                + Transformation.constants('d_7');
            % or constant
            %botHeight = 51.0;
            botHeight = 45.0;
        end
    end
end
    