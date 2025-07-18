classdef RollBot < BaseBot
    properties
        mode = 'roll';
        maxStepSize = 2;
        wheelRadius = 6;
        %maxWheelVelocity = 2;
        wheelShift = [-pi/4 pi/4 3*pi/4 -3*pi/4];
        maxSteeringVelocity = deg2rad(5);
        enableVelocityLimit = false;
        randomSteeringSlip = 0.05;
        enableJointLimit = true;
        jointLimitsHip = [deg2rad(90) deg2rad(150)];
        enableBetaWheel = true;
        adjustBotHeight = false;    % not done
        
        maxOrientation = deg2rad(0);
    end
    properties(Constant)
        a = 3*sqrt(2)+(31/2);
        b = 6+(31*sqrt(2)/2);
    end
    methods
        function obj = RollBot(terrain, path)
            obj@BaseBot();
            %obj.maxI = 300;
            
            % load functions
            addpath('transformations/functions2');
            obj.functionEndPositions = @tbe_roll_end_pos;
            obj.functionAByWheels = @aby_roll_wheels;
            obj.functionJointPositions = @tbe_roll_joint_pos;

            % initial joints ?
            obj.waist = [0 0 0 0];
            obj.hip = [deg2rad(120) deg2rad(120) deg2rad(120) deg2rad(120)];
            obj.knee = [0 0 0 0];
            obj.steering = [deg2rad(90) deg2rad(90) deg2rad(90) deg2rad(90)];
            obj.contact = [0 0 0 0];
            
            %{
            % init position
            initPosition = obj.get_init_position(terrain, path);
            % initialize state
            initState = obj.init_state(initPosition, terrain);
            obj = obj.add_history(initState);
            %}
            
            %obj.kinematics = RollKinematics();
            obj.costAnalyzer = RollAnalyzer();
        end
        function obj = initialize(obj, terrain, path, startX)
            disp("ROLL BOT INITALIZE METHOD ************");
            obj = obj.clear_history();
            % initialize state
            initState = obj.init_state(terrain, path, startX);
            initState = obj.init_roll(initState, terrain);
            obj = obj.add_history(initState);
            
            % set height
            %obj.botHeight = obj.get_bot_height(initState);
        end
        function state = init_state2(obj, terrain, path, initPosition)
            disp("Roll init state")
            state = BotState();
            state.basePosition = initialPosition;
            state.baseOrientation = [0 0 deg2rad(0)];
            %state.baseOrientation = [0 0 deg2rad(90)];
            state.dotBaseOrientation = [0 0 0];
            state.dotEndOrientations = zeros(4,3);
            state.anglesHip = [deg2rad(120) deg2rad(120) deg2rad(120) deg2rad(120)];
            state.anglesWaist = [0 0 0 0];
            state.anglesKnee = [0 0 0 0];
            state.anglesSteering = [deg2rad(90) deg2rad(90) deg2rad(90) deg2rad(90)];
            %state.anglesSteering = [0 0 0 0];
            state.anglesContact = [0 0 0 0];    
            % fix init Z
            state.basePosition(3) = obj.get_bot_height(state) + initialPosition(3);
            state.endPositions = obj.get_global_end_positions(state);
            % init roll
            state = obj.init_roll(state, terrain);
            state.clear_dots();
        end
        function firstState = init_roll(obj, firstState, terrain)
            disp('init roll')
            oldState = firstState;
            elevationDifferenceWheels = terrain.get_elevations(firstState.endPositions(:,1),firstState.endPositions(:,2)) ...
                - firstState.endPositions(:,3);
            elevationDifferenceBase = mean(elevationDifferenceWheels);
            firstState.dotBasePosition(3) = elevationDifferenceBase;
            firstState.dotEndPositions(:,3) = elevationDifferenceWheels;
            firstState = update_hip(obj, firstState, oldState);
            % update base position
            firstState.basePosition = oldState.basePosition + firstState.dotBasePosition;
        end
        function obj = move_next_step(obj, moveVector, turnAngle, terrain, path)
            previousState = obj.get_last_state();
            newState = BotState(previousState);
            
            % calculate new values and update state
            newState.endPositions = obj.get_global_end_positions(newState);
            
            % global steering ???
            %newState.anglesSteering = [0 0 0 0];
            % update vectors
            newState.dotBaseOrientation = [0 0 turnAngle];
            %newState.dotBaseOrientation = [0 0 0];
            %newState.dotBasePosition(1:2) = moveVector;
            newState.dotBasePosition(1:2) = obj.rotate_vector(moveVector, turnAngle+newState.baseOrientation(3));
            %{
            disp('TURN ANGLE');
            disp(rad2deg(turnAngle));
            disp('dot_xy =====================');
            disp(newState.dotBasePosition(1:2));
            disp(newState.dotEndPositions(:,1:2));
            %}
                        
            % update steering
            sigma = obj.wheelShift;
            dot_x_base = newState.dotBasePosition(1);
            dot_y_base = newState.dotBasePosition(2);
            dot_gamma_base = newState.dotBaseOrientation(3);
            sin_steer = dot_x_base + obj.a.*dot_gamma_base;
            cos_steer = -(dot_y_base + obj.a.*dot_gamma_base);
            local_steering = deg2rad(90);
            if cos_steer ~= 0
                local_steering = ones(1,4).*atan2(sin_steer, cos_steer);
            end
            steering_hat = local_steering;
            % random [-1, 1]
            r = (2).*rand(1,1) + 1;
            slip = r.*steering_hat.*obj.randomSteeringSlip;
            %steering_hat = steering_hat + r.*steering_hat.*obj.randomSteeringSlip;
            %steering_hat = obj.get_local_to_global_steering(local_steering, previousState.baseOrientation(3));
            newState.dotAnglesSteering = steering_hat - previousState.anglesSteering;
            
            % zDots = terrainZ - AxlePosition
            [x1, y1, x2, y2] = obj.getWheelEnds(newState, turnAngle+newState.baseOrientation(3));
            dot_beta_wheel = terrain.get_angle_elevations(x1, y1, x2, y2);
            dot_beta_wheel(dot_beta_wheel > obj.maxOrientation) = obj.maxOrientation;
            dot_beta_wheel(dot_beta_wheel < -obj.maxOrientation) = -obj.maxOrientation;
            dot_z_wheel = terrain.get_elevations(newState.endPositions(:,1),newState.endPositions(:,2)) ...
                - newState.endPositions(:,3);
            if obj.enableBetaWheel == true
                dot_z_wheel = dot_z_wheel + obj.b.*cos(steering_hat)'.*dot_beta_wheel;
            end
            dot_z_base = mean(dot_z_wheel);
            new_bot_height = obj.get_bot_height(newState);
            if obj.adjustBotHeight == true & new_bot_height > obj.botHeight
                dot_z_base = dot_z_base.*1.5;
            end
            newState.dotEndPositions(:,3) = dot_z_wheel;
            newState.dotBasePosition(3) = dot_z_base;
            
            % dot end positions
            dot_x_wheel = (dot_x_base + obj.a.*dot_gamma_base)./(sin(steering_hat)');
            dot_y_wheel = [0 0 0 0];
            hip_wheel = newState.anglesHip;
            %dot_beta_wheel = newState.endOrientations(:,2);
            % get dot_beta_wheel from terrain?
            if obj.enableBetaWheel == true
                dot_x_wheel = dot_x_wheel + (24*cos(hip_wheel)'-21).*dot_beta_wheel;
            end
            newState.dotEndPositions(:,1) = dot_x_wheel;
            newState.dotEndPositions(:,2) = dot_y_wheel;
            
            % steering velocity limit
            if obj.enableVelocityLimit == true
                newState.dotAnglesSteering(newState.dotAnglesSteering > deg2rad(obj.maxSteeringVelocity)) = deg2rad(obj.maxSteeringVelocity);
                newState.dotAnglesSteering(newState.dotAnglesSteering < deg2rad(-obj.maxSteeringVelocity)) = deg2rad(-obj.maxSteeringVelocity);

                dot_gamma_base(dot_gamma_base > obj.maxSteeringVelocity) = obj.maxSteeringVelocity;
                dot_gamma_base(dot_gamma_base < -obj.maxSteeringVelocity) = -obj.maxSteeringVelocity;
                dot_x_wheel(dot_x_wheel > obj.maxWheelVelocity) = obj.maxWheelVelocity;
            end
            %newState.dotAnglesSteering = dot_gamma_base;
            new_steering_hat = newState.anglesSteering + newState.dotAnglesSteering;
                        
            % assume dot_gamma_wheel = 0
            dot_gamma_wheel = [0 0 0 0];
            new_dot_gamma_base = dot_gamma_wheel(1) - newState.dotAnglesSteering(1);
            %dot_gamma_base = new_dot_gamma_base;
            old_dot_base_position = newState.dotBasePosition(:);
            new_dot_x_base = sin(steering_hat).*dot_x_wheel + ...
                cos(steering_hat).*dot_y_wheel - obj.a.*dot_gamma_base + slip;
            new_dot_y_base = -cos(steering_hat).*dot_x_wheel + ...
                sin(steering_hat).*dot_y_wheel - obj.a.*dot_gamma_base + slip;
            if obj.enableBetaWheel == true
                new_dot_x_base = sin(steering_hat).*dot_x_wheel+ ...
                    sin(steering_hat).*(21-24*cos(hip_wheel)).*dot_beta_wheel' - obj.a.*dot_gamma_base;
                new_dot_y_base = cos(steering_hat).*dot_x_wheel + ...
                    cos(steering_hat).*(21-24*cos(hip_wheel)).*dot_beta_wheel' + obj.a.*dot_gamma_base;
                new_dot_y_base = -new_dot_y_base;
                dot_alpha_base = (1/4).*sum(cos(steering_hat).*dot_beta_wheel');
                dot_beta_base = (1/4).*sum(sin(steering_hat).*dot_beta_wheel');
                %{
                disp('new base xy ab');
                disp(new_dot_x_base);
                disp(new_dot_y_base);
                disp(dot_alpha_base);
                disp(dot_beta_base);
                %}
            end
            newState.dotBasePosition(1) = new_dot_x_base(1);
            newState.dotBasePosition(2) = new_dot_y_base(1);
            newState.dotBaseOrientation(3) = dot_gamma_base;
            %{
            disp('SLIP');
            disp(slip);
            disp('new dot xy base');
            disp(newState.dotBasePosition(1:2));
            disp('dot gamma wheel');
            disp(dot_gamma_wheel);
            disp('dot gamma base');
            disp(rad2deg(dot_gamma_base));
            disp('new dot gamma base');
            disp(rad2deg(new_dot_gamma_base));
            disp('dot base ori');
            disp(rad2deg(newState.dotBaseOrientation));
            %}
            
            % equation base orientaion
            eq_alpha = cos(steering_hat)'.*dot_beta_wheel;
            eq_beta = sin(steering_hat)'.*dot_beta_wheel;
            % expected base orientation
            expected_alpha = obj.get_expected_alpha(newState);
            expected_beta = obj.get_expected_beta(newState);
            %{
            disp('equation alpha/beta ==========')
            disp(eq_alpha);
            disp(eq_beta);
            disp(mean(eq_alpha));
            disp(mean(eq_beta));
            disp('expected alpha/beta ================')
            disp(expected_alpha);
            disp(expected_beta);
            %}
            eq_alpha = mean(eq_alpha);
            eq_beta = mean(eq_beta);
            newState.dotBaseOrientation(1) = eq_alpha;
            newState.dotBaseOrientation(2) = eq_beta;
            
            % update hip
            newState = obj.update_hip(newState, previousState);
            % update base position
            newState.basePosition = previousState.basePosition + newState.dotBasePosition;
            newState.endPositions = obj.get_global_end_positions(newState);
            newState.baseOrientation = previousState.baseOrientation + newState.dotBaseOrientation;
            newState.anglesSteering = newState.anglesSteering + newState.dotAnglesSteering;
            
            
            %newState.baseOrientation(newState.baseOrientation > maxOrientation) = maxOrientation;
            %newState.baseOrientation(newState.baseOrientation < -maxOrientation) = -maxOrientation;
            
            % calculations
            %{
            hip = newState.anglesHip;
            dot_hip = newState.dotAnglesHip;
            dot_x_wheels = newState.dotEndPositions(:,1);
            dot_y_wheels = newState.dotEndPositions(:,2);
            dot_alpha_wheels = newState.dotEndOrientations(:,1);
            dot_beta_wheels = newState.dotEndOrientations(:,2);
            dot_x_base = sin(steering_hat)*dot_x_wheels + cos(steering_hat)*dot_y_wheels - a*(gamma_wheels + dot_steering);
            dot_y_base = -cos(steering_hat)*dot_x_wheels + sin(steering_hat)*dot_y_wheels - a*(gamma_wheels + dot_steering);
            dot_z_base = dot_z_wheels + b*(sin(steering)*dot_alpha_wheel + cos(steering)*dot_beta_wheels) + 48*sin(hip)*dot_hip;
            %}
            
            % add new state to history
            obj = obj.add_history(newState);
        end
        function [x1, y1, x2, y2] = getWheelEnds(obj, newState, orientation)
            wheelVectorPos = obj.rotate_vector([obj.wheelRadius 0], orientation);
            wheelVectorNeg = obj.rotate_vector([-obj.wheelRadius 0], orientation);
            x0 = newState.endPositions(:,1);
            y0 = newState.endPositions(:,2);
            x1 = x0 + wheelVectorNeg(1);
            x2 = x0 + wheelVectorPos(1);
            y1 = y0 + wheelVectorNeg(2);
            y2 = y0 + wheelVectorPos(2);
            
        end
        %% Helper Functions
        function newState = update_hip(obj, newState, previousState)
            % get equations & evaluate
            matrix_function = obj.functionAByWheels;
            [A_eval, By_eval] = matrix_function(newState.anglesHip, newState.anglesSteering, ...
                newState.dotEndPositions, newState.dotEndOrientations, ...
                newState.dotBasePosition, newState.dotBaseOrientation);
            knownVector = inv(A_eval)*By_eval;
            
            % update hip joints
            newState.dotAnglesHip = knownVector(1:4,1)';
            
            %newState.dotBaseOrientation(:,1) = knownVector(5:8,1)';
            %newState.dotBaseOrientation(:,2) = knownVector(9:12,1)';
            newState.anglesHip = previousState.anglesHip + newState.dotAnglesHip;
            if obj.enableJointLimit == true
                newState.anglesHip(newState.anglesHip < obj.jointLimitsHip(1)) = obj.jointLimitsHip(1);
                newState.anglesHip(newState.anglesHip > obj.jointLimitsHip(2)) = obj.jointLimitsHip(2);
            end
            
            % update steering ??
            %newState.dotAnglesSteering = knownVector(:,:);
            %newState.anglesSteering = previousState.anglesSteering + newState.dotAnglesSteering;
        end
        function directions = get_global_to_local_steering(obj, anglesSteering, baseOrientationGamma)
            % all legs, 4x1 vector
            % from SILO: GlobalSteeringAngle*leg = gamma_base + steering_leg + shift_leg
            shift = [-pi/4 pi/4 3*pi/4 -3*pi/4];
            directions = anglesSteering - baseOrientationGamma - shift;
        end
        function directions = get_local_to_global_steering(obj, anglesSteering, baseOrientationGamma)
            % all legs, 4x1 vector
            % from SILO: GlobalSteeringAngle*leg = gamma_base + steering_leg + shift_leg
            shift = [-pi/4 pi/4 3*pi/4 -3*pi/4];
            directions = anglesSteering + baseOrientationGamma + shift;
        end
        
        
    end
end
    