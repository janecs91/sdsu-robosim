classdef WalkBot < LeggedBot
    properties
        mode = 'walk';
        sampleStepPoints = 5;
        % motion of legs
        % row = motion phase #
        % column = legs involved
        legMotionOrder = [1; 3; 2; 4];
        initialActiveLeg = 2;
        numLegs = 4;
        numWheels = 0;
        useAltKinematics = false;
        useAltStepAdjuster = true;
    end
    methods
        function obj = WalkBot(terrain, path)
            swingStrategy = 1;
            obj@LeggedBot(swingStrategy);
            %obj.useAltKinematics = false;
            %obj.stepAdjusterSwing = StepAdjusterSwing(obj.stepSafetyValue,2);
            %obj.stepAdjusterStance = StepAdjusterStance(obj.stepSafetyValue,2);
            
            % load functions
            addpath('transformations/functions2');
            obj.functionEndPositions = @tbe_walk_end_pos;
            obj.functionsRotLeg = {@tbe_walk_rot_leg_1, @tbe_walk_rot_leg_2, @tbe_walk_rot_leg_3, @tbe_walk_rot_leg_4};
            obj.functionsAByLeg = {@aby_walk_leg_1, @aby_walk_leg_2, @aby_walk_leg_3, @aby_walk_leg_4};
            obj.functionsAByBody = {@aby_walk_body_1, @aby_walk_body_2, @aby_walk_body_3, @aby_walk_body_4};
            obj.functionJointPositions = @tbe_walk_joint_pos;
 
            % initial joints ?
            obj.waist = [deg2rad(45) deg2rad(-45) deg2rad(45) deg2rad(-45)];
            obj.hip = [deg2rad(120) deg2rad(120) deg2rad(120) deg2rad(120)];
            obj.knee = [deg2rad(60) deg2rad(60) deg2rad(60) deg2rad(60)];
            obj.steering = [0 0 0 0];
            obj.contact = [0 0 0 0];
            
            %{
            % init position
            initPosition = obj.get_init_position(terrain, path);
            % initialize state
            [initState, obj] = obj.init_state(initPosition, terrain);
            obj = obj.add_history(initState);
            obj = obj.init_leg_bot(initState);
            %}
        end
        function obj = initialize(obj, terrain, path, startX)
            disp("WALK BOT INITALIZE METHOD ************");
            obj = obj.clear_history();
            % initialize state
            initState = obj.init_state(terrain, path, startX);
            initState = obj.init_walk(initState, obj.maxStepSize, terrain);
            %initState.clear_dots();
            obj = obj.add_history(initState);
            %obj = obj.init_leg_bot(initState);
            
            % set height
            obj.botHeight = obj.get_bot_height(initState);
            obj.stepAdjusterHeight.originalBaseZ = obj.get_max_height_from_waist_to_foot(initState);
        end
        %{
        function [state, obj] = init_state2(obj, terrain, path, initPosition, initGamma, waist, hip, knee, steering, contact)
            disp("Walk init state")
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
            state.anglesWaist = [deg2rad(45) deg2rad(-45) deg2rad(45) deg2rad(-45)];
            state.anglesHip = [deg2rad(120) deg2rad(120) deg2rad(120) deg2rad(120)];
            state.anglesKnee = [deg2rad(60) deg2rad(60) deg2rad(60) deg2rad(60)];
            state.anglesSteering = [0 0 0 0];
            state.anglesContact = [0 0 0 0];
            % fix init Z
            state.basePosition(3) = obj.get_init_bot_height() + initPosition(3);
            %state.basePosition(3) = obj.get_bot_height(state) + initialPosition(3);
            %state.endPositions = obj.get_global_end_positions(state);
            % init walk
            state = obj.init_walk(state, obj.maxStepSize, terrain);
            % reset dot values
            state.clear_dots();
        end
        %}
        function firstState = init_walk(obj, firstState, stepSize, terrain)
            disp('init walk')
            firstState = obj.init_legs(firstState, stepSize, terrain);
        end
    end
end
    