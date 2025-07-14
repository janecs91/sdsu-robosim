classdef StepAdjuster
    properties
        adjustType = 0;
        safetyValue = 0.9;  % mu
        bodyScale = 0.9;
        legMax = 48;
        
        useConstantTestValues0 = false;
        constantTurnAngle0 = deg2rad(0);
        constant_dx_b0 = 20;
        %constant_dx_i0 = 10;
    end
    methods
        %{
        function obj = StepAdjuster()
        end
        %}
        function result = is_stable_step_size(obj, jointPositions, stepSize, z_new, legLength)
            % test stability
            % (x_ankle+((3/4)x_stepsize)-x_hip)^2 + (z_ankle-z_hip)^2 < (a3+a4)^2
            % l = length of leg = a3+a4 = 24+24 = 48 cm
            i_hip = 3;
            i_ankle = 5;
            x_ankle = jointPositions(i_ankle, 1);
            x_hip = jointPositions(i_hip, 1);
            z_hip = jointPositions(i_hip, 3);
            maxStableLength = obj.invBodyStepRatio*(sqrt(legLength^2-(z_new-z_hip)^2)-(x_ankle-x_hip));
            result = stepSize < maxStableLength;
        end
        function [relativeLegVector, globalLegVector, relativeBodyVector] = get_stable_step_vector(obj, previousState, activeLeg, maxStepVector, turnAngle, terrain, legLength)
            bestStepVector = maxStepVector;
            bodyVector = bestStepVector./obj.numLegs;
            stepSizes = bestStepVector(1):obj.stepDecrement:obj.minStepSize;
            if obj.isStepAdjustable == false
                stepSizes = bestStepVector(1);
            end
            for i=1:length(stepSizes)
                bestStepVector(1) = stepSizes(i);
                
                [turnAngleLeg, legDistance, futureEndPosition] = obj.predict_end_position(activeLeg, bestStepVector, turnAngle);
                %futureJointPositions = obj.get_global_joint_positions(futureState, activeLeg);
                %futureEndPositions = obj.get_global_end_positions(futureState);
                %futureEndPosition = futureEndPositions(activeLeg,:);
                
                %fprintf("leg vector %0.2f %0.2f rotated leg vector %0.2f %0.2f TA %.2f\n", legVector(1:2), rotatedLegVector(:), rad2deg(turnAngle));
                futureTerrainElevation = terrain.get_elevation(futureEndPosition(1), futureEndPosition(2));
                %futureTerrainElevation = futureTerrainElevations(activeLeg);
                previousEndPositions = obj.get_global_end_positions(previousState);
                %{
                elevationDifference = futureTerrainElevations - previousEndPositions(:,3);
                relativeLegVector(3) = elevationDifferences(activeLeg);
                globalLegVector(3) = relativeLegVector(3);
                %}
                
                %is_stable = true;
                is_stable = obj.is_stable_step_size(futureJointPositions, relativeLegVector(1), futureTerrainElevation, legLength);
                
                %% alt stable?
                %jointPositions = obj.get_global_joint_positions(previousState, activeLeg);
                %dz_i = stepVector(1);
                %is_stable2 = obj.get_leg_step_vector(state, leg, turnAngle, legLength, jointPositions, dz_i);
                if is_stable
                    break
                end
                %disp('reducing stepsize')
            end
            relativeBodyVector = bestStepVector/obj.numLegs;
            %relativeBodyVector(3) = relativeLegVector(3);
            relativeBodyVector(3) = mean(elevationDifferences);
            xBV = relativeBodyVector(3);
            % check leg overextension
            disp("FJP **************************")
            disp(futureJointPositions);
            basePositionZ = previousState.basePosition(3)+relativeBodyVector(3);
            prevFootPositionZ = previousEndPositions(activeLeg,3);
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
            fprintf("leg %d relBodyVectorMean %.2f basePosZ %.2f prevFoot %.2f footPos %.2f baseFootDiff %.2f overExtDiff %.2f newBodyVector %.2f \n\n", ...
                activeLeg, xBV, basePositionZ, prevFootPositionZ, footPositionZ, baseToFootDifference, ...
                overextendedDifference, relativeBodyVector(3));
        end
        %% apply any limitations on body vector here
        function relativeBodyVector = adjust_body_vector(obj, previousState, activeLeg, relativeBodyVector)
            if activeLeg == 1
                disp("MIN BODY VECTOR LIMIT LEG 1 ==========")
                obj.lastPhaseMinimumBodyVector(1:2) = relativeBodyVector(1:2);
            end
            if obj.enableBodyVectorLimit && activeLeg ~= 1
                disp("MIN BODY VECTOR LIMIT LEG 2 ==========")
                obj.lastPhaseMinimumBodyVector(1:2) = min(relativeBodyVector(1:2), ...
                    obj.lastPhaseMinimumBodyVector(1:2));
                relativeBodyVector(1:2) = obj.lastPhaseMinimumBodyVector;
            end
            
            disp('after phase limit - relative body vector ===============================')
            disp(relativeBodyVector);
            % check bodyVector
            if obj.enableBodyVectorLimit
                % update for walking, check minimum of front two legs
                stanceLegs = setdiff(1:obj.numLegs, activeLeg);
                ankleFromWaistDistance = obj.getAnkleFromWaistDistanceAlongX(previousState, stanceLegs(1));
                maxStableAnkleFromWaistDistance = (ankleFromWaistDistance-obj.minAnkleToWaistDistanceLimit)/obj.numLegs;
                relativeBodyVector(1) = min(relativeBodyVector(1), maxStableAnkleFromWaistDistance);
                disp('ankle-waist distance')
                disp(ankleFromWaistDistance);
                disp(maxStableAnkleFromWaistDistance);
                disp('new body vector after ankle/waist distance')
                disp(relativeBodyVector);
            end
        end
        
        %% alt
        function [mu_dx_b, mu_dy_b, dz_b] = get_base_step_vector(obj, state, leg, turnAngle, terrain)
            % sigma = slope of path
            % slope
            altLeg = mod(leg,2)+1;
            jointPositions = obj.get_local_joint_positions(state, altLeg);
            jointPositions = jointPositions{1};
            max_dx_b = obj.get_distance_waist_to_foot(state, altLeg);
            
            z_a = jointPositions(end,3);
            x_a = jointPositions(end,1);
            y_a = jointPositions(end,2);
            a=obj.a_0+(sqrt(2)/2)*obj.a_2;
            
            
            lengthFromTurnAngle = cos(turnAngle)*x_a - sin(turnAngle)*y_a;
            if lengthFromTurnAngle > 2*a
                disp("BIGGER THAN 2A!!!!!!!!!!!");
            end
            dx_b0 = 2*a - lengthFromTurnAngle;
            dx_b1 = max_dx_b;
            dx_b = dx_b1;
            dx_b = dx_b - obj.minBodyToSwingAnkleDistance;
            dx_b = dx_b - obj.minBodyToKneeDistance;
            dy_b = tan(turnAngle)*dx_b;
            mu_dx_b = dx_b*obj.bodyScale*obj.safetyValue;
            mu_dy_b = dy_b*obj.bodyScale*obj.safetyValue;
            if obj.useConstantTestValues0
                mu_dx_b = obj.constant_dx_b0;
            end
            
            x_center = (1/3)*(-2*a+cos(turnAngle)*x_a+sin(turnAngle)*y_a+dx_b);
            y_center = (1/3)*(-sin(turnAngle)*x_a+cos(turnAngle)*y_a);
            
            x_b_prime = state.basePosition(1)+mu_dx_b;
            y_b_prime = state.basePosition(2)+mu_dy_b;
            previousTerrainElevation = terrain.get_elevation(state.basePosition(1), state.basePosition(2));
            newTerrainElevation = terrain.get_elevation(x_b_prime, y_b_prime);
            botHeight = obj.get_bot_height(state);
            dz_b = newTerrainElevation-previousTerrainElevation;
            expectedBotHeight = abs(dz_b)+botHeight;
            disp('previousTerrainElevation')
            disp(previousTerrainElevation);
            disp('newTerrainElevation')
            disp(newTerrainElevation)
            disp('botHeight');
            disp(botHeight);
            disp('expectedBotHeight');
            disp(expectedBotHeight);
            if expectedBotHeight < 46
                dz_b = 0;
            end
            
            %% debug
            a_rel1 = a/sqrt(2);
            a_rel2 = x_a - y_a;
            disp("DX parts ==========")
            fprintf("a %.2f a_rel1 %.2f a_rel2 %.2f x_a %.2f y_a %.2f z_a %.2f\n", a, a_rel1, a_rel2, x_a, y_a, z_a);
            fprintf("tan a: %.2f\n", tan(turnAngle));
            fprintf("dx_b=%.2f mu*dx_b=%.2f\n", dx_b0, mu_dx_b);
            fprintf("dy_b=%.2f mu*dy_b=%.2f\n", dy_b, mu_dy_b);
            fprintf("max_dx_b (distance ankle to waist):%.2f\n", max_dx_b);
            
        end
        function [dx_i, localLegVector] = get_leg_step_vector(obj, state, leg, slope, turnAngle, terrain, path)
            [dx_i, localLegVector] = obj.adjust_leg_step_binary_search(state, slope, turnAngle, leg, terrain, path);
        end
        function stepSize = get_max_step_size(obj, slope, x_a, x_h, z_a, z_h, gamma)
            %stepSize = sqrt(obj.safetyValue*(obj.legLength^2 - (z_a - z_h)^2)/(1+slope^2)) - (x_a-x_h);
            stepSize = obj.legLength;           
        end
        function [x_a_prime, y_a_prime, z_a_prime] = get_new_prime(obj, x_a, y_a, z_a, gamma, dx)
            rotatedStepVector = obj.rotateVector([dx 0], gamma);
            x_a_prime = x_a + rotatedStepVector(1);
            y_a_prime = y_a + rotatedStepVector(2);
            z_a_prime = 0;
        end
        function [x_a_prime, y_a_prime, z_a_prime] = get_new_prime_alt(obj, x_a, y_a, z_a, gamma, dx)
            % not used
            slope = gamma;
            x_a_prime = x_a + dx;
            c = 0;
            y_a_prime = slope*(dx+x_a) + c;
            z_a_prime = 0;
        end
        function isStable = is_stable_alt(obj, state, leg, x_a, y_a, z_a, x_a_prime, y_a_prime, z_a_prime)
            jointPositions = obj.get_local_joint_positions(state, leg);
            jointPositions = jointPositions{1};
            x_h = jointPositions(3,1);
            y_h = jointPositions(3,2);
            z_h = jointPositions(3,3);
            numerator = [-x_a_prime-obj.a_0 y_a_prime-obj.a_0];
            denominator = [y_a_prime-obj.a_0 x_a_prime-obj.a_0];
            waist_hat = atan2(numerator(leg), denominator(leg));
            new_waist = waist_hat - deg2rad(45);
            disp('calc waist')
            disp(x_a_prime-x_a);
            disp(y_a_prime-y_a);
            disp('new waist')
            disp(rad2deg(new_waist));
            % swing
            newState = copy(state);
            oldWaist = newState.anglesWaist(leg);
            %newState.anglesWaist(leg) = newState.anglesWaist(leg) + new_waist;
            newState.anglesWaist(leg) = new_waist;
            fprintf("STEP SIZE -- WAIST HAT: %.2f Old Waist: %.2f New Waist: %.2f\n", rad2deg(waist_hat), ...
                rad2deg(oldWaist), rad2deg(newState.anglesWaist(leg)));
            
            % stance
            altLegs = setdiff(1:obj.numLegs, leg);
            jointPositions = obj.get_local_joint_positions(newState, leg);
            jointPositions = jointPositions{1};
            x_h_prime = jointPositions(3,1);
            y_h_prime = jointPositions(3,2);
            z_h_prime = jointPositions(3,3);
            stepSize = (x_a_prime-x_h_prime)^2 + (y_a_prime-y_h_prime)^2 + (z_a_prime-z_h_prime)^2;
            isStable = stepSize < obj.legLength^2;
            disp("STABILITY VALUES ==")
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
            fprintf("stepSizeValue: %.2f \n", stepSize);
            fprintf("l^2: %.2f", obj.legLength^2);
        end
        function [stepSize, localLegVector] = adjust_leg_step_binary_search(obj, state, slope, turnAngleBody, leg, terrain, path)
            gamma = turnAngleBody + state.baseOrientation(3);
            jointPositions = obj.get_local_joint_positions(state, leg);
            jointPositions = jointPositions{1};
            x_a = jointPositions(end,1);
            y_a = jointPositions(end,2);
            z_a = jointPositions(end,3);
            %x_h = jointPositions(3,1);
            %z_h = jointPositions(3,3);
            vector = [10 0 0];
            testState = copy(state);
            testState.basePosition = state.basePosition + vector;
            endPos = obj.get_global_end_positions(testState, leg);
            oldZ = endPos(3);
            %maxStepSize = obj.get_max_step_size(slope, x_a, x_h, z_a, z_h, gamma);
            maxStepSize = obj.legLength/2;
            minStepSize = maxStepSize/10;
            stepSizes = linspace(maxStepSize, minStepSize, 10);
            fprintf("-- max stepsize: %.2f\n", maxStepSize);
            stepAdjust = obj.legLength/2;
            stepSize = 0;
            bodyVector = [20 0 0];
            [futurePoint, futurePathIndex] = obj.get_next_path_point(state, path);
            while abs(stepAdjust) > 1
                %stepSize = stepSizes(i);
                stepSize = stepSize + stepAdjust;
                bodyVector = [stepSize 0 0];
                fprintf("-- trying bodysize: %.2f\n", bodyVector);
                fprintf("-- trying stepsize: %.2f\n", stepSize);
                fprintf("-- step adjusted by: %.2f\n", stepAdjust);
                [localLegVector, futureEndPosition] = obj.predict_leg_vector(state, leg, bodyVector, turnAngleBody);
                %[x_a_prime, y_a_prime, z_a_prime] = obj.get_new_prime(x_a, y_a, z_a, gammaLeg, legDistance);
                x_a_prime = futureEndPosition(1);
                y_a_prime = futureEndPosition(2);
                terrainElevation = terrain.get_elevation(x_a_prime, y_a_prime);
                %z_a_prime = terrainElevation;
                z_a_prime = z_a + (terrainElevation - oldZ);
                fprintf("oldZ %.2f terrainElevation %.2f z_a %.2f z_a_prime %.2f\n", oldZ, ...
                    terrainElevation, z_a, z_a_prime);
                z_a_prime = z_a;
                isStable = obj.is_stable_alt(testState, leg, x_a, y_a, z_a, x_a_prime, y_a_prime, z_a_prime);
                sign = -1;
                if isStable
                    disp("stable")
                    sign = 1;
                    %break
                end
                stepAdjust = sign*stepAdjust / 2;
            end
            stepSize = min(obj.safetyValue*stepSize,35);
            fprintf("*********************************\n");
            fprintf("----------found stable stepsize: %.2f\n", stepSize);
            fprintf("----------last step adjust: %.2f\n", stepAdjust);
        end
        function [localLegVector, futureEndPosition] = predict_leg_vector(obj, previousState, activeLeg, bodyVector, turnAngle)
            %turnAngle = deg2rad(-5);
            previousEndPosition = obj.get_global_end_positions(previousState, activeLeg);
            point1 = previousEndPosition(1:2);
            
            %% futureState
            futureEndPosition = obj.get_future_end_position(previousState, activeLeg, bodyVector, turnAngle);
            
            %% evaluate
            localLegVector = obj.get_leg_vector_to_point(previousState, activeLeg, futureEndPosition);
            
            disp("ANGLES **********************************")
            disp('bodyvec');
            disp(bodyVector);
            disp('end pos (prev, then future)');
            disp(previousEndPosition);
            disp(futureEndPosition);
            
            %{
            bodyVectorLength = phaseBodyVector(1)./obj.numLegs;
            relativeLegVector = [distance - bodyVectorLength 0 0];
            relativeLegVector = obj.rotateVector(relativeLegVector(1:2), turnAngleLeg);
            %}
        end
    end
end