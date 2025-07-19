classdef PullBot < LeggedBot
    properties
        mode = 'pull';
        % motion of legs
        % row = motion phase #
        % column = legs involved
        legMotionOrder = [1; 2];
        initialActiveLeg = 1;
        numLegs = 2;
        numWheels = 2;
        useAltKinematics = true;
    end
    methods
        function obj = PullBot(terrain, path)
            swingStrategy = 1;
            obj@LeggedBot(swingStrategy);
            
            % load functions
            addpath('transformations/functions2');
            obj.functionEndPositions = @tbe_pull_end_pos;
            obj.functionAByWheels = @aby_pull_wheels;
            obj.functionsRotLeg = {@tbe_pull_rot_leg_1, @tbe_pull_rot_leg_2};
            obj.functionsAByLeg = {@aby_pull_leg_1, @aby_pull_leg_2};
            obj.functionsAByBody = {@aby_pull_body_1, @aby_pull_body_2};
            obj.functionJointPositions = @tbe_pull_joint_pos;
            
            % kinematics
            obj.kinematics = WalkKinematics();
            obj.costAnalyzer = WalkAnalyzer();
            
            % initial joints ?
            obj.waist = [deg2rad(45) deg2rad(-45) 0 0];
            obj.hip = [deg2rad(90) deg2rad(90) deg2rad(120) deg2rad(120)];
            obj.knee = [deg2rad(90) deg2rad(90) deg2rad(0) deg2rad(0)];
            obj.steering = [0 0 -3*pi/4 3*pi/4];
            obj.contact = [0 0 0 0];
            
            %{
            % initialize state
            initState = obj.init_state(obj, terrain, path, initialPosition);
            initState = obj.init_pull(initState, obj.maxStepSize, terrain);
            obj = obj.add_history(initState);
            obj = obj.init_leg_bot(initState);
            
            % set height
            obj.botHeight = obj.get_bot_height(initState);
            obj.stepAdjusterHeight.originalBaseZ = obj.get_max_height_from_waist_to_foot(initState);
            %}
        end
        function obj = initialize(obj, terrain, path, startX)
            disp("PULL BOT INITALIZE METHOD ************");
            obj = obj.clear_history();
            % initialize state
            initState = obj.init_state(terrain, path, startX);
            initState = obj.init_pull(initState, obj.maxStepSize, terrain);
            obj = obj.add_history(initState);
            %obj = obj.init_leg_bot(initState);
            
            % set height
            obj.botHeight = obj.get_bot_height(initState);
            obj.stepAdjusterHeight.originalBaseZ = obj.get_max_height_from_waist_to_foot(initState);
        end
        function firstState = init_pull(obj, firstState, stepSize, terrain)
            disp('init pull')
            firstState = obj.init_legs(firstState, stepSize, terrain);
            % adjust wheels, check elevation too
            firstState = obj.update_wheels(firstState, terrain, 0);
        end
        %{
        function state = init_state(obj, terrain, path, initPosition, initGamma, waist, hip, knee, steering, contact)
            state = BotState();
            state.basePosition = initialPosition;
            [angle, slope] = path.get_gamma_at_index(1);
            state.baseOrientation = [0 0 angle];
            state.dotBaseOrientation = [0 0 0];
            state.dotEndOrientations = zeros(4,3);
            % waist range: -45 to 45 (side-front)
            state.anglesWaist = [deg2rad(45) deg2rad(-45) 0 0];
            %state.anglesWaist = [deg2rad(25) deg2rad(-25) 0 0];
            % hip range: 0 (straight up) 90 (flat) to 120 (orig) 180 (down)
            state.anglesHip = [deg2rad(90) deg2rad(90) deg2rad(120) deg2rad(120)];
            % knee: 0 (folded) 60 (orig) 180(stretched)
            % orig paper knee: [60 60 0 0]
            state.anglesKnee = [deg2rad(90) deg2rad(90) deg2rad(0) deg2rad(0)];
            %state.anglesKnee = [0 0 0 0];
            state.anglesSteering = [0 0 -3*pi/4 3*pi/4];
            state.anglesContact = [0 0 0 0];
            % fix init Z
            %obj.botHeight = obj.get_bot_height(state)+10;
            obj.botHeight = obj.get_bot_height(state);
            state.basePosition(3) = obj.botHeight + obj.get_lowest_point(state, terrain);
            %state.basePosition(3) = 39;
            state.endPositions = obj.get_global_end_positions(state);
            % init pull
            
            % reset dot values
            state.clear_dots();
        end
        %}
        
        
        %% Pull Override?
        function newState = body_sequence(obj, newState, activeLeg, bodyVectorI, turnAngleBodyI, stancePathIndex, terrain)
            if obj.verbose
                disp('pull body sequence');
            end
            newState = obj.update_body(newState, activeLeg, bodyVectorI, turnAngleBodyI, stancePathIndex);
            newState = obj.update_wheels(newState, terrain, turnAngleBodyI);
        end
        function newState = update_wheels(obj, previousState, terrain, turnAngleBody)
            % calculate new elevations for wheels
            if obj.verbose
                disp("moving wheels...")
            end
            previousEndPositions = obj.get_global_end_positions(previousState);
            newTerrainElevations = terrain.get_elevations(previousEndPositions(3:4,1), previousEndPositions(3:4,2));
            dotZWheels =  newTerrainElevations - previousEndPositions(3:4,3);
            
            % set values
            tempState = BotState(previousState);
            tempState.dotBasePosition = [0 0 0];
            tempState.dotBaseOrientation = [0 0 0];
            tempState.dotEndPositions(3:4,3) = dotZWheels;
            
            % get equations and evaluate
            matrix_function = obj.functionAByWheels;
            [A_eval, By_eval] = matrix_function(tempState.anglesHip, tempState.anglesSteering, ...
                tempState.dotEndPositions, tempState.dotEndOrientations, ...
                tempState.dotBasePosition, tempState.dotBaseOrientation);
            knownVector = inv(A_eval)*By_eval;
            
            % update hip joints
            newState = BotState(previousState);
            newState.dotAnglesHip(3:4) = knownVector(1:2,1);
            newState.anglesHip(3:4) = previousState.anglesHip(3:4) + newState.dotAnglesHip(3:4);
            newState.dotEndPositions(3:4,:) = tempState.dotEndPositions(3:4,:);
            
            % update steering?
            newState.dotAnglesSteering(3:4) = turnAngleBody - previousState.anglesSteering(3:4);
            newState.anglesSteering(3:4) = turnAngleBody;
        end
    end
    methods(Static)
    end
end
    