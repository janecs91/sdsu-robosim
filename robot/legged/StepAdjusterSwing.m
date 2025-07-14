classdef StepAdjusterSwing < StepAdjuster
    properties
        % limit ranges
        minDxi = 0;
        frontCornerDistance = 0;
        strategy;
        allowNegativeDxi = false;

        %% old properties
        adjustType = 0;
        bodyScale = 0.9;
        useConstantTestValues = false;
        constant_dx_i = 0;
        useConstantTestValues0 = false;
        constantTurnAngle0 = deg2rad(0);
        constant_dx_b0 = 20;
        %constant_dx_i0 = 10;
    end
    %{
%% ROBOT
The joint and link lengths are as follows.
a_0=d_0=15.5 cm (half width of the robot body)
d_1=15 cm (half height of the robot body)
a_2=6 cm (distance from robot body to first joint)
a_3=a_4=24 cm (leg length)
    %}
    methods
        function obj = StepAdjusterSwing(safetyValue, strategy)
            fprintf("SAFETY VALUE SWING: %d", safetyValue)
            obj@StepAdjuster(safetyValue);
            obj.frontCornerDistance = sqrt(2)*15.5;     % How was this calculated?
            %obj.backCornerDistance = sqrt(2)*15.5;      % Is this correct?
            %obj.minDxi = obj.frontCornerDistance;
            obj.minDxi = 0;
            
            strategies = {SwingTriangle() SwingPose()};
            obj.strategy = strategies{strategy};
            fprintf("Using strategy %s\n", obj.strategy.strategyName);
        end
        function [relativeLegVector] = get_step_vector_main(obj, bot, forwardState, activeLeg, terrain, path)
            if activeLeg < 5
                [planned_dx_i, planned_dy_i, planned_dz_i] = obj.get_step_vector2(bot, forwardState, activeLeg, terrain, path);
            else
                % Ignore for now
                % Try old strategy for legs 3 and 4
                [planned_dx_i, planned_dy_i, planned_dz_i] = obj.get_step_vector1(bot, forwardState, activeLeg, terrain, turnAngle);
            end
            localLegVector = [planned_dx_i planned_dy_i];
            if obj.useConstantTestValues
                planned_dx_i = obj.constant_dx_i;
            end
            relativeLegVector = [localLegVector 0];
        end
        function [mu_dx_i, mu_dy_i, mu_dz_i] = get_step_vector2(obj, bot, state, leg, terrain, path)
            %% This is for leg step vector: LEGS 1 and 2
            %% Compare this with StepAdjusterOld.get_stable_step_vector()
            % Returns relative leg vector
            endPositions = bot.get_global_joint_positions(state, leg);
            endPositions2 = state.endPositions(leg,:);
            %disp("global end pts")
            %disp(endPositions)
            %disp(endPositions2);
            state = bot.update_end_positions(state);
            %disp("after update")
            %disp(state.endPositions(leg,:));
            jointPositions = bot.get_local_joint_positions(state, leg);
            jointPositions = jointPositions{1};
            waist_x = jointPositions(2,1);
            z_a = jointPositions(end,3);
            x_a = jointPositions(end,1);
            y_a = jointPositions(end,2);
            z_h = jointPositions(end,3);
            x_h = jointPositions(end,1);
            y_h = jointPositions(end,2);
            distance_waist_to_foot = bot.get_distance_waist_to_foot(state, leg);
            %% Possibly update to backCornerDistance for back legs (if trying for legs 3 and 4)
            % Possibly use 1/2 leg length ??
            dx_i_front = (obj.maxLegLength-distance_waist_to_foot)+obj.frontCornerDistance;
            dx_i_back = distance_waist_to_foot*obj.safetyValue;
            %dx_i_back = distance_waist_to_foot;
            %dx_i = obj.maxLegLength + obj.frontCornerDistance;
            %dx_i = obj.maxLegLength;
            dx_i = dx_i_front;
            if leg > 2
                dx_i = dx_i_back;
            end
            dz_i = 0;
            testRange = dx_i:-1:obj.minDxi;
            %{
            fprintf("find swing step: leg %d - x_a: %.2f, y_a: %.2f, z_a: %.2f\n", leg, x_a, y_a, z_a);
            fprintf("x_h: %.2f, y_h: %.2f, z_h: %.2f\n", x_h, y_h, z_h);
            fprintf("local waist x: %.2f, distance_waist_to_foot: %.2f, maxdx_i: %.2f\n", waist_x, distance_waist_to_foot, dx_i);
            disp("testRange")
            disp(testRange);
            %}
            for i=testRange
                %fprintf("* test range i: %d\n", i);
                moveX = i;
                globalFootPoint = obj.strategy.get_next_global_foot_point(bot, state, leg, terrain, path, moveX);
                newTerrainZ = terrain.get_elevation(globalFootPoint(1), globalFootPoint(2));
                globalFootPoint(3) = newTerrainZ;
                localFootPoint = bot.change_global_to_local(state, globalFootPoint);
                localOldFootPoint = bot.change_global_to_local(state, state.endPositions(leg,1:2));
                %fprintf("state pos: %.2f %.2f, gamma: %.2f\n", state.basePosition(1:2), rad2deg(state.baseOrientation(3)));
                %fprintf("(global) old ft pt: %.2f %.2f %.2f, new ft pt: %.2f %.2f %.2f\n", ...
                %    state.endPositions(leg,:), globalFootPoint);
                %fprintf("(local) old ft pt: %.2f %.2f, local old2: %.2f %.2f %.2f, new ft pt: %.2f %.2f %.2f\n", ...
                %    [x_a y_a], localOldFootPoint, localFootPoint);
                
                x_a_prime = localFootPoint(1);
                y_a_prime = localFootPoint(2);
                z_a_prime = localFootPoint(3);
                [x_h, y_h, z_h, x_h_prime, y_h_prime, z_h_prime] = StepAdjusterSwing.get_hips(bot, state, leg, x_a_prime, y_a_prime, z_a_prime);
                %{
                fprintf("trying leg %d swing step - moveX: %.2f\n", leg, moveX)
                fprintf("x_a_prime: %.2f, y_a_prime: %.2f, z_a_prime: %.2f\n", x_a_prime, y_a_prime, z_a_prime);
                fprintf("x_h_prime: %.2f, y_h_prime: %.2f, z_h_prime: %.2f\n", x_h_prime, y_h_prime, z_h_prime);
                %}
                isStable = StepAdjusterSwing.check_stability_swing(bot, state, leg, x_a, y_a, z_a, ...
                    x_a_prime, y_a_prime, z_a_prime, obj.maxLegLength);
                %fprintf("stable? %d\n", isStable);
                if isStable
                    %{
                    disp("found stable leg step")
                    fprintf("x_a: %.2f, y_a: %.2f, z_a: %.2f\n", x_a, y_a, z_a);
                    fprintf("x_h: %.2f, y_h: %.2f, z_h: %.2f\n", x_h, y_h, z_h);
                    fprintf("x_a_prime: %.2f, y_a_prime: %.2f, z_a_prime: %.2f\n", x_a_prime, y_a_prime, z_a_prime);
                    fprintf("x_h_prime: %.2f, y_h_prime: %.2f, z_h_prime: %.2f\n", x_h_prime, y_h_prime, z_h_prime);
                    %}
                    %fprintf("x_a_prime: %.2f, x_h_prime: %.2f, xap-xhp: %.2f\n", x_a_prime, x_h_prime, abs(x_a_prime-x_h_prime)); 
                    %fprintf("y_a_prime: %.2f, y_h_prime: %.2f, yap-yhp: %.2f\n", y_a_prime, y_h_prime, abs(y_a_prime-y_h_prime)); 
                    %fprintf("z_a_prime: %.2f, z_h_prime: %.2f, zap-zhp: %.2f\n", z_a_prime, z_h_prime, abs(z_a_prime-z_h_prime)); 
                
                    break
                end
            end
            dx_i = x_a_prime-x_a;
            dy_i = y_a_prime-y_a;
            %fprintf("stable moveX for leg %d: %.2f\n", leg, moveX);
            %fprintf("dx_i: %.2f, dy_i: %.2f\n", dx_i, dy_i);
            mu_dx_i = max(obj.safetyValue*dx_i, 0);
            if obj.allowNegativeDxi == true
                mu_dx_i = dx_i;
            end
            %mu_dy_i = dy_i;
            mu_dy_i = obj.safetyValue*dy_i;
            mu_dz_i = dz_i;
        end

        %% OLD METHODS
        function result = is_stable_step_size(obj, bot, jointPositions, stepSize, z_new, legLength)
            %% OLD METHOD
            % test stability
            % (x_ankle+((3/4)x_stepsize)-x_hip)^2 + (z_ankle-z_hip)^2 < (a3+a4)^2
            % l = length of leg = a3+a4 = 24+24 = 48 cm
            i_hip = 3;
            i_ankle = 5;
            x_ankle = jointPositions(i_ankle, 1);
            x_hip = jointPositions(i_hip, 1);
            z_hip = jointPositions(i_hip, 3);
            maxStableLength = bot.invBodyStepRatio*(sqrt(legLength^2-(z_new-z_hip)^2)-(x_ankle-x_hip));
            result = stepSize < maxStableLength;
        end
        function newFakeState = predict_end_position(obj, bot, activeLeg, bestStepVector, turnAngle)
            %% OLD METHOD
            state = bot.get_last_state();
            dx_b = bestStepVector(1);
            dy_b = bestStepVector(2);
            gamma = turnAngle;
            update_end_positions_bool = true;
            newFakeState = bot.get_new_state_given_vector_and_gamma(state, dx_b, dy_b, gamma, update_end_positions_bool);
        end
        function [mu_dx_i, mu_dy_i, mu_dz_i] = get_step_vector1(obj, bot, previousState, activeLeg, terrain, turnAngle)
            %% MODIFIED OLD METHOD
            %% This is for leg step vector: LEGS 3 and 4
            % Check if this is used for walking legs 1 and 2?
            % [mu_dx_i, mu_dy_i, mu_dz_i] = get_step_vector2(obj, bot, state, leg, terrain, path)
            legLength = obj.maxLegLength;
            maxStepVector = [bot.maxStepSize 0 0]; 
            bestStepVector = maxStepVector;
            stepSizes = bestStepVector(1):bot.stepDecrement:bot.minStepSize;
            if bot.isStepAdjustable == false
                stepSizes = bestStepVector(1);
            end
            for i=1:length(stepSizes)
                bestStepVector(1) = stepSizes(i);
                %% Fix here
                futureState = obj.predict_end_position(bot, activeLeg, bestStepVector, turnAngle);    % Need to fix this method
                % old original method call - [turnAngleLeg, legDistance, futureEndPosition] = obj.predict_end_position(activeLeg, bestStepVector, turnAngle);
                futureJointPositions = bot.get_global_joint_positions(futureState, activeLeg);
                disp("*** Future Joint Positions")
                disp(futureJointPositions)
                futureEndPositions = bot.get_global_end_positions(futureState);
                futureEndPosition = futureEndPositions(activeLeg,:);
                
                %fprintf("leg vector %0.2f %0.2f rotated leg vector %0.2f %0.2f TA %.2f\n", legVector(1:2), rotatedLegVector(:), rad2deg(turnAngle));
                futureTerrainElevation = terrain.get_elevation(futureEndPosition(1), futureEndPosition(2));
                previousEndPositions = bot.get_global_end_positions(previousState);
                previousEndPosition = previousEndPositions(activeLeg, :);
                previousEndPositionZ = previousEndPosition(3);
                
                elevationDifference = futureTerrainElevation - previousEndPositionZ;
                bestStepVector(3) = elevationDifference;
                %globalLegVector(3) = relativeLegVector(3);
                
                
                %is_stable = true;
                is_stable = obj.is_stable_step_size(bot, futureJointPositions, bestStepVector(1), futureTerrainElevation, legLength);

                fprintf("TESTING SWING leg #%d => stepVector %f | is_stable %d | prevFootPos (%.2f %.2f %.2f) | newFootPos (%.2f %.2f %.2f)  \n\n", ...
                activeLeg, bestStepVector(1), is_stable, previousEndPosition, futureEndPosition);
                
                %% alt stable?
                %jointPositions = bot.get_global_joint_positions(previousState, activeLeg);
                %dz_i = stepVector(1);
                %is_stable2 = obj.get_leg_step_vector(state, leg, turnAngle, legLength, jointPositions, dz_i);
                if is_stable
                    break
                end
                %disp('reducing stepsize')
            end
            % Do I need base to foot difference? Probably for overextending
            % on extreme terrain differences
            futureTerrainElevations = terrain.get_elevations(futureEndPositions(:, 1), futureEndPositions(:, 2));
            elevationDifferences = futureTerrainElevations-futureEndPositions(:, 3);
            relativeBodyVector = bestStepVector/bot.numLegs;
            %relativeBodyVector(3) = relativeLegVector(3);
            relativeBodyVector(3) = mean(elevationDifferences);
            xBV = relativeBodyVector(3);
            % check leg overextension
            basePositionZ = previousState.basePosition(3)+relativeBodyVector(3);
            footPositionZ = futureTerrainElevation;
            baseToFootDifference = abs(basePositionZ-footPositionZ);
            overextendedDifference = 0;
            if baseToFootDifference > legLength
                disp("OVEREXTENDED ******************************************");
                overextendedDifference = baseToFootDifference - legLength;
                if relativeLegVector(3) < 0
                    overextendedDifference = -overextendedDifference;
                end
                relativeBodyVector(3) = relativeBodyVector(3)+overextendedDifference;
            end
            disp("========================================");
            disp("========================================");
            fprintf("SWING FINAL leg #%d => relBodyVectorMean %.2f basePosZ %.2f prevFootPos (%.2f %.2f %.2f) newFootPos (%.2f %.2f %.2f) baseFootDiff %.2f overExtDiff %.2f newBodyVectorz %.2f \n\n", ...
                activeLeg, xBV, basePositionZ, previousEndPosition, futureEndPosition, baseToFootDifference, ...
                overextendedDifference, relativeBodyVector(3));
            % mu_dx_i, mu_dy_i, mu_dz_i = relativeLegVector
            relativeLegVectorCell = num2cell(bestStepVector);
            [mu_dx_i, mu_dy_i, mu_dz_i] = relativeLegVectorCell{:};
            fprintf("Relative leg vector for leg #%d: %.2f %.2f %.2f", activeLeg, mu_dx_i, mu_dy_i, mu_dz_i)
            %% Is this really relative? check if global or relative
        end
    end
    methods(Static)
        %% Helper Methods
        function [x_h, y_h, z_h, x_h_prime, y_h_prime, z_h_prime] = get_hips(bot, state, leg, x_a_prime, y_a_prime, z_a_prime)
            %% OK for 4 legs
            jointPositions = bot.get_local_joint_positions(state, leg);
            jointPositions = jointPositions{1};
            x_h = jointPositions(3,1);
            y_h = jointPositions(3,2);
            z_h = jointPositions(3,3);
            gamma = 0;
            new_waist = WalkKinematics.get_waist_given_prime(bot, leg, x_a_prime, y_a_prime, gamma);
            [x_h_prime, y_h_prime, z_h_prime] = WalkKinematics.get_hip_location_given_waist(bot, state, new_waist, leg);
            %fprintf("leg %d - old waist: %.2f, new waist: %.2f\n", leg, rad2deg(state.anglesWaist(leg)), rad2deg(new_waist));
        end
        
        %% Stability Check Helper Methods
        function isStable = check_stability_leg_length(x_a_prime, y_a_prime, z_a_prime, ...
                x_h_prime, y_h_prime, z_h_prime, legLength)
            %% OK for 4 legs
            stepSize = (x_a_prime-x_h_prime)^2 + (y_a_prime-y_h_prime)^2 + (z_a_prime-z_h_prime)^2;
            isStable = stepSize < legLength^2;
            %fprintf("stability leg length - x_a_prime: %.2f, x_h_prime: %.2f, xap-xhp: %.2f\n", x_a_prime, x_h_prime, abs(x_a_prime-x_h_prime)); 
            %fprintf("stability leg length - %.2f <? %.2f\n", stepSize, legLength^2); 
        end
        function isStable = check_boundary(bot, leg, x_a_prime, y_a_prime)
            %% Update for legs 3 and 4
            sideCheck2 = [1 1];
            sideCheck3 = [1 1];
            sideCheck1 = [(y_a_prime>-x_a_prime) (y_a_prime>-x_a_prime)];
            sideCheck2 = [(y_a_prime<x_a_prime) (y_a_prime<x_a_prime)];
            sideCheck3 = [(x_a_prime>bot.a_0) (x_a_prime>bot.a_0)];
            isStable = sideCheck1(leg)&sideCheck2(leg)&sideCheck3(leg);
            %{
            fprintf("sideCheck1: %d\n", sideCheck1(leg));
            fprintf("sideCheck2: %d\n", sideCheck2(leg));
            fprintf("sideCheck3: %d\n", sideCheck3(leg));
            %}
        end
        function isStable = check_joint_angles(bot, state, leg, x_a, y_a, x_a_prime, y_a_prime)
            %% Update for legs 3 and 4
            % add z???
            moveVector = [x_a_prime-x_a y_a_prime-y_a];
            newTestState = copy(state);
            newTestState.activeLeg = leg;
            newTestState.dotBasePosition = [0 0 0];
            newTestState.dotBaseOrientation = [0 0 0];
            newTestState.dotEndPositions = zeros(4,3);
            newTestState.dotEndPositions(leg,1:2) = moveVector;
            % have eval kinematics fail silently??
            [dot_waist, dot_hip, dot_knee, newWaist, newHip, newKnee] = bot.eval_alt_kinematics_swing(newTestState);
            waistCheck = [(-90<newWaist)&(newWaist<0) (0<newWaist)&(newWaist<90)];
            kneeCheck = [(0<newHip)&(newHip<180) (0<newHip)&(newHip<180)];
            hipCheck = [(90<newKnee)&(newKnee<180) (0<newKnee)&(newKnee<90)];
            isStable = waistCheck(leg)&kneeCheck(leg)&hipCheck(leg);
            
        end
        function isStable = check_stability_swing(bot, state, leg, x_a, y_a, z_a, ...
                x_a_prime, y_a_prime, z_a_prime, legLength)
            %fprintf("swing stability checks - leg %d\n", leg);
            [x_h, y_h, z_h, x_h_prime, y_h_prime, z_h_prime] = StepAdjusterSwing.get_hips(bot, state, leg, x_a_prime, y_a_prime, z_a_prime);
            legLengthCheck = StepAdjusterSwing.check_stability_leg_length(x_a_prime, y_a_prime, z_a_prime, ...
                x_h_prime, y_h_prime, z_h_prime, legLength);
            
            extraCheck = 0;     % We are ignoring the boundary and joint angles check
            % other checks
            isStable = legLengthCheck;
            boundaryCheck = 0;
            jointCheck = 0;
            if legLengthCheck == true && extraCheck == true
                boundaryCheck = StepAdjusterSwing.check_boundary(bot, state, leg, x_a, y_a, x_a_prime, y_a_prime);
                jointCheck = StepAdjusterSwing.check_joint_angles(bot, state, leg, x_a, y_a, x_a_prime, y_a_prime);
                isStable = isStable&boundaryCheck&jointCheck;
                
            end
            %{
            fprintf("isStableLegLength: %d\n", legLengthCheck);
            fprintf("boundaryCheck: %d\n", boundaryCheck);
            fprintf("jointCheck: %d\n", jointCheck);
            fprintf("isStable: %d\n", isStable);
            
            fprintf("STABILITY VALUES == leg: %d\n",leg)
            fprintf("x_a: %.2f x_a_prime %.2f\n", x_a, x_a_prime);
            fprintf("x_h: %.2f x_h_prime %.2f\n", x_h, x_h_prime);
            fprintf("y_a: %.2f y_a_prime %.2f\n", y_a, y_a_prime);
            fprintf("y_h: %.2f y_h_prime %.2f\n", y_h, y_h_prime);
            fprintf("z_a: %.2f z_a_prime %.2f\n", z_a, z_a_prime);
            fprintf("z_h: %.2f z_h_prime %.2f\n", z_h, z_h_prime);
            fprintf("xa-xhprime: %.2f ya-yhprime %.2f za-zhprime: %.2f\n", x_a_prime-x_h_prime, ...
                y_a_prime-y_h_prime, z_a_prime-z_h_prime);
            fprintf("sq [] xa-xhprime: %.2f ya-yhprime %.2f za-zhprime: %.2f\n", (x_a_prime-x_h_prime)^2, ...
                (y_a_prime-y_h_prime)^2, (z_a_prime-z_h_prime)^2);
            %}
            %fprintf("stepSizeValue: %.2f \n", stepSize);
            %fprintf("l^2: %.2f", bot.legLength^2);
        end
        
    end
end