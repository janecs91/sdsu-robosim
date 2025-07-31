classdef StepAdjusterStance < StepAdjuster
    properties
        verbose = false;
        % limit ranges
        minDxb = 0;
        keepBaseHeightConstant = true;
    end
    methods
        function obj = StepAdjusterStance(safetyValue)
            obj@StepAdjuster(safetyValue);
        end
        function [mu_dx_b, mu_dy_b, mu_dz_b, turnBodyAngle, futurePathIndex] = get_step_vector2(obj, bot, state, activeLeg, terrain, path)
            stanceLegs = setdiff(1:bot.numLegs, activeLeg);
            nextState = copy(state);
            nextState.activeLeg = activeLeg;
            nextLeg = bot.get_next_leg_number(nextState);
            stanceLeg = nextLeg;

            state = bot.update_end_positions(state);
            oldJointPositions = bot.get_local_joint_positions(state);
            oldJointPositionsStanceLeg = oldJointPositions{stanceLeg};
            z_a = oldJointPositionsStanceLeg(end,3);
            x_a = oldJointPositionsStanceLeg(end,1);
            y_a = oldJointPositionsStanceLeg(end,2);
            
            footDistanceFromHip = bot.get_distance_hip_to_foot(state, stanceLeg);
            %dx_b = max(footDistanceFromHip-bot.a_0,0);
            dx_b = max(footDistanceFromHip, 0);
            %dx_b = dx_b/2;
            if stanceLeg > 2
                footDistanceFromWaist = bot.get_distance_waist_to_foot(state, stanceLeg);
                allowableLegStretch = max(obj.maxLegLength-footDistanceFromWaist,0);
                allowableLegStretchWithSafety = allowableLegStretch.*obj.safetyValue;
                if obj.verbose
                    fprintf("footDistanceFromHip:%d, footDistanceFromWaist: %d, allowableLegStretch: %d, allowableLegStretchWithSafety: %d\n", ...
                    footDistanceFromHip, footDistanceFromWaist, allowableLegStretch, allowableLegStretchWithSafety);
                end
                dx_b = min(footDistanceFromHip,allowableLegStretch);
            end
            %bodyVectorDivisor = max(ceil(bot.numLegs/2),1);
            %bodyVectorDivisor = 1;
            %fprintf("bodyVectorDivisor: %d\n",bodyVectorDivisor);
            %dx_b = dx_b/bodyVectorDivisor;
            %{
            fprintf("stance leg: %d\n", stanceLeg);
            fprintf("footdistfrombase: %d\n", max(footDistanceFromBase-(2*bot.a_0),0));
            fprintf("footdistfromfront: %d\n", max(footDistanceFromFront-bot.a_0,0));
            %}
            %dx_b = 24;
            %disp("joint positions stance")
            %disp(jointPositions);
            %{
            disp("stance stability check");
            fprintf("stance step leg %d - x_a: %.2f, fdw: %.2f, maxdx_b: %.2f\n", stanceLeg, x_a, footDistanceFromFront, dx_b);
            fprintf("leg %d - x_a: %.2f , bot2a: %.2f-- initial dx_B to test: %.2f\n", stanceLeg, ...
                x_a, 2*bot.a_0, dx_b); 
            %}
            testRange = dx_b:-1:obj.minDxb;
            %disp("TEST RANGE")
            %disp(testRange);
            triedPathPts = zeros(size(testRange,2),2);
            c = 0;
            futurePathPoint = state.endPositions(stanceLeg,:);
            for i=testRange
                c = c+1;
                moveX = i;
                %fprintf("i: %.2f, moveX:%.2f\n",i, moveX);
                [futurePathPoint, futurePathIndex] = bot.get_next_path_point(state, path, moveX);
                %fprintf("Leg %d - FT PATH PT: %.2f %.2f ; PATH IDX: %d\n", stanceLeg, futurePathPoint, futurePathIndex);
                triedPathPts(c,:) = futurePathPoint;
                [angle1, slope1] = path.get_gamma_at_index(futurePathIndex);
                fakeState = copy(state);
                fakeState.basePosition(1:2) = futurePathPoint(1:2);
                [futurePathPoint2, futurePathIndex2] = bot.get_next_path_point(fakeState, path, moveX);
                [angle2, slope2] = path.get_gamma_at_index(futurePathIndex2);
                %fprintf("angle 1: %.2f, angle 2: %.2f\n", rad2deg(angle1), rad2deg(angle2));
                %turnBodyAngle = angle1 - state.baseOrientation(3);
                turnBodyAngle = angle2 - state.baseOrientation(3);
                %turnBodyAngle = 0;
                %fprintf("current gamma: %.2f, turn body angle: %.2f\n", rad2deg(state.baseOrientation(3)), rad2deg(turnBodyAngle));
                localPathPoint = bot.change_global_to_local(state, futurePathPoint);
                dx_b = localPathPoint(1);
                dy_b = localPathPoint(2);
                %[distance, angle] = bot.get_vector_to_point(state.basePosition(1:2), futurePathPoint);
                %turnBodyAngle = angle - state.baseOrientation(3);
                isStable = StepAdjusterStance.check_stability_stance(stanceLeg, turnBodyAngle, x_a, y_a, dx_b, dy_b);
                if bot.numLegs > 2
                    % check both legs 3 and 4
                    % leg 3
                    newJointPositionsAll = bot.get_local_joint_positions(fakeState);
                    oldJointPositionsStanceLeg = oldJointPositions{3};
                    z_a3 = oldJointPositionsStanceLeg(end,3);
                    x_a3 = oldJointPositionsStanceLeg(end,1);
                    y_a3 = oldJointPositionsStanceLeg(end,2);
                    newJointPositions = newJointPositionsAll{3};
                    x_h_prime3 = newJointPositions(3,1);
                    y_h_prime3 = newJointPositions(3,2);
                    z_h_prime3 = newJointPositions(3,3);
                    isStableLeg3 = obj.check_stability_leg_length(x_a3, y_a3, z_a3, x_h_prime3, y_h_prime3, z_h_prime3, obj.maxLegLength);
                    % leg 4
                    oldJointPositionsStanceLeg = oldJointPositions{4};
                    z_a4 = oldJointPositionsStanceLeg(end,3);
                    x_a4 = oldJointPositionsStanceLeg(end,1);
                    y_a4 = oldJointPositionsStanceLeg(end,2);
                    newJointPositions = newJointPositionsAll{4};
                    x_h_prime4 = newJointPositions(3,1);
                    y_h_prime4 = newJointPositions(3,2);
                    z_h_prime4 = newJointPositions(3,3);
                    isStableLeg4 = obj.check_stability_leg_length(x_a4, y_a4, z_a4, x_h_prime4, y_h_prime4, z_h_prime4, obj.maxLegLength);
                    isStable = isStable && isStableLeg3 && isStableLeg4;
                    if obj.verbose
                        fprintf("is stable back legs for dx_b(%d)? isStableLeg3: %d, isStableLeg4: %d\n", dx_b, isStableLeg3, isStableLeg4);
                    end
                end
                if isStable
                    if obj.verbose
                        disp("found stable body step stance")
                        fprintf("stance leg %d - dx_b: %.2f, dy_b:%.2f\n", stanceLeg, dx_b, dy_b);
                    end
                    break
                end
                if i==size(testRange,2)
                    disp("could not find isstable stance, end of check")
                    fprintf("last values leg %d - dx_b: %.2f, dy_b:%.2f\n", stanceLeg, dx_b, dy_b);
                    dx_b = 0;
                    dy_b = 0;
                    fprintf("reset stance leg %d - dx_b: %.2f, dy_b:%.2f\n", stanceLeg, dx_b, dy_b);
                end
            end
            %dz_b = obj.get_base_z_vector(bot, state, terrain, futurePathPoint);
            dz_b = obj.get_base_z_vector2(bot, state, terrain, futurePathPoint);
            %dz_b = 0;
            mu_dx_b = max(dx_b, obj.minDxb);
            mu_dy_b = dy_b;
            mu_dz_b = dz_b;
            %{
            disp("CURR GLOB PT")
            disp(state.basePosition);
            disp("TRIED PATH PTS")
            disp(triedPathPts(1:c,:));
            %}
        end
        
        function dz_b = get_base_z_vector(obj, bot, state, terrain, futurePoint)
            %% not in use
            waistZ = bot.get_waist_z(state);
            % get difference in terrain
            baseZ = state.basePosition(3);
            oldTerrainZ = terrain.get_elevation(state.basePosition(1), state.basePosition(2));
            newTerrainZ = terrain.get_elevation(futurePoint(1), futurePoint(2));
            differenceInTerrain = newTerrainZ - oldTerrainZ;
            dz_b_min = 0;
            dzFromBaseBottomToTerrain = waistZ - newTerrainZ;
            if dzFromBaseBottomToTerrain < bot.minBottomZFromTerrain
                dz_b_min = bot.minBottomZFromTerrain-dzFromBaseBottomToTerrain;
            end
            dz_b = dz_b_min + differenceInTerrain;
            %{
            disp("GETTING BASE Z VECTOR======")
            fprintf("waistZ: %.2f, dz_waist_terrain: %.2f, dz_b_min: %.2f\n", waistZ, dzFromBaseBottomToTerrain, dz_b_min);
            fprintf("old base pos: %.2f, old bot height: %.2f, base-botheight: %.2f\n", ...
                state.basePosition(3), bot.botHeight, state.basePosition(3) - bot.botHeight);
            fprintf("old terrain: %.2f, new terrain: %.2f, new-old: %.2f\n", oldTerrainZ, newTerrainZ, ...
                differenceInTerrain)
            fprintf("dz_b: %.2f\n", dz_b)
            %}
            %dz_b = 0;
        end
        function dz_b = get_base_z_vector2(obj, bot, state, terrain, futurePoint)
            baseZ = state.basePosition(3);
            oldTerrainZ = terrain.get_elevation(state.basePosition(1), state.basePosition(2));
            newTerrainZ = terrain.get_elevation(futurePoint(1), futurePoint(2));
            frontBasePosition = futurePoint(1)+bot.a_0;
            backBasePosition = futurePoint(1)-bot.a_0;
            highestElevation = terrain.get_highest_elevation(backBasePosition,futurePoint(2),frontBasePosition,futurePoint(2),1);
            lowestPoint = bot.get_lowest_point(state, terrain);
            averageElevation = bot.get_average_elevation(state, terrain);
            dz_b = 0;
            newTerrainZ = averageElevation;
            if obj.keepBaseHeightConstant
                dz_b = newTerrainZ+bot.minBottomZFromTerrain-baseZ;
            else
                minimumBaseZ = newTerrainZ+bot.minBottomZFromTerrain;
                maximumBaseZ = newTerrainZ+bot.maxBottomZFromTerrain;
                if baseZ < minimumBaseZ && false
                    dz_b = minimumBaseZ-baseZ;
                end
                if baseZ > maximumBaseZ && false
                    dz_b = maximumBaseZ-baseZ;
                end
            end
            
            %dz_b = max(highestElevation, lowestPoint)-oldTerrainZ;
            if obj.verbose || true
                fprintf("baseZ: %d, oldTerrainZ: %d, newTerrainZ: %d\n", baseZ, oldTerrainZ, newTerrainZ);
                fprintf("highestElevation: %d, lowest point:%d, dz_b: %.2f\n", highestElevation, lowestPoint, dz_b);
            end
        end

        %% old methods
        %% apply any limitations on body vector here
        function relativeBodyVector = adjust_body_vector(obj, bot, previousState, activeLeg, relativeBodyVector)
            if activeLeg == 1
                %disp("MIN BODY VECTOR LIMIT LEG 1 ==========")
                bot.lastPhaseMinimumBodyVector(1:2) = relativeBodyVector(1:2);
            end
            if bot.enableBodyVectorLimit && activeLeg ~= 1
                %disp("MIN BODY VECTOR LIMIT LEG 2 ==========")
                bot.lastPhaseMinimumBodyVector(1:2) = min(relativeBodyVector(1:2), ...
                    bot.lastPhaseMinimumBodyVector(1:2));
                relativeBodyVector(1:2) = bot.lastPhaseMinimumBodyVector(1:2);
            end
            
            %disp('after phase limit - relative body vector ===============================')
            %disp(relativeBodyVector);
            % check bodyVector
            if bot.enableBodyVectorLimit
                % update for walking, check minimum of front two legs
                stanceLegs = setdiff(1:bot.numLegs, activeLeg);
                ankleFromWaistDistance = obj.getAnkleFromWaistDistanceAlongX(previousState, stanceLegs(1));
                maxStableAnkleFromWaistDistance = (ankleFromWaistDistance-obj.minAnkleToWaistDistanceLimit)/bot.numLegs;
                relativeBodyVector(1) = min(relativeBodyVector(1), maxStableAnkleFromWaistDistance);
                %{
                disp('ankle-waist distance')
                disp(ankleFromWaistDistance);
                disp(maxStableAnkleFromWaistDistance);
                disp('new body vector after ankle/waist distance')
                disp(relativeBodyVector);
                %}
            end
        end

        %% alt
        function [mu_dx_b, mu_dy_b, dz_b] = get_base_step_vector(bot, state, leg, turnAngle, terrain)
            % sigma = slope of path
            % slope
            altLeg = mod(leg,2)+1;
            jointPositions = bot.get_local_joint_positions(state, altLeg);
            jointPositions = jointPositions{1};
            max_dx_b = obj.get_distance_waist_to_foot(state, altLeg);
            
            z_a = jointPositions(end,3);
            x_a = jointPositions(end,1);
            y_a = jointPositions(end,2);
            a=bot.a_0+(sqrt(2)/2)*bot.a_2;
            
            
            lengthFromTurnAngle = cos(turnAngle)*x_a - sin(turnAngle)*y_a;
            if lengthFromTurnAngle > 2*a
                disp("BIGGER THAN 2A!!!!!!!!!!!");
            end
            dx_b0 = 2*a - lengthFromTurnAngle;
            dx_b1 = max_dx_b;
            dx_b = dx_b1;
            dx_b = dx_b - bot.minBodyToSwingAnkleDistance;
            dx_b = dx_b - bot.minBodyToKneeDistance;
            dy_b = tan(turnAngle)*dx_b;
            mu_dx_b = dx_b*bot.bodyScale*obj.safetyValue;
            mu_dy_b = dy_b*bot.bodyScale*obj.safetyValue;
            if bot.useConstantTestValues0
                mu_dx_b = bot.constant_dx_b0;
            end
            
            x_center = (1/3)*(-2*a+cos(turnAngle)*x_a+sin(turnAngle)*y_a+dx_b);
            y_center = (1/3)*(-sin(turnAngle)*x_a+cos(turnAngle)*y_a);
            
            x_b_prime = state.basePosition(1)+mu_dx_b;
            y_b_prime = state.basePosition(2)+mu_dy_b;
            previousTerrainElevation = terrain.get_elevation(state.basePosition(1), state.basePosition(2));
            newTerrainElevation = terrain.get_elevation(x_b_prime, y_b_prime);
            botHeight = bot.get_bot_height(state);
            dz_b = newTerrainElevation-previousTerrainElevation;
            expectedBotHeight = abs(dz_b)+botHeight;
            %{
            disp('previousTerrainElevation')
            disp(previousTerrainElevation);
            disp('newTerrainElevation')
            disp(newTerrainElevation)
            disp('botHeight');
            disp(botHeight);
            disp('expectedBotHeight');
            disp(expectedBotHeight);
            %}
            if expectedBotHeight < 46
                dz_b = 0;
            end
            
            %% debug
            a_rel1 = a/sqrt(2);
            a_rel2 = x_a - y_a;
            %{
            disp("DX parts ==========")
            fprintf("a %.2f a_rel1 %.2f a_rel2 %.2f x_a %.2f y_a %.2f z_a %.2f\n", a, a_rel1, a_rel2, x_a, y_a, z_a);
            fprintf("tan a: %.2f\n", tan(turnAngle));
            fprintf("dx_b=%.2f mu*dx_b=%.2f\n", dx_b0, mu_dx_b);
            fprintf("dy_b=%.2f mu*dy_b=%.2f\n", dy_b, mu_dy_b);
            fprintf("max_dx_b (distance ankle to waist):%.2f\n", max_dx_b);
            %}
        end
    end
    methods(Static)
        %% Stability Check Helper Methods
        function isStable = check_stability_stance(leg, turnAngle, x_a, y_a, dx_b, dy_b)
            stabilityCheckLeg1Phrase = y_a - ((sin(turnAngle)-cos(turnAngle))/(sin(turnAngle)+cos(turnAngle)))*(x_a - dx_b);
            stabilityCheckLeg2Phrase = y_a + ((sin(turnAngle)+cos(turnAngle))/(sin(turnAngle)-cos(turnAngle)))*(x_a - dx_b);
            stabilityCheckLeg1 = dy_b < y_a - ((sin(turnAngle)-cos(turnAngle))/(sin(turnAngle)+cos(turnAngle)))*(x_a - dx_b);
            stabilityCheckLeg2 = dy_b > y_a + ((sin(turnAngle)+cos(turnAngle))/(sin(turnAngle)-cos(turnAngle)))*(x_a - dx_b);
            stabilityCheck = [stabilityCheckLeg1 stabilityCheckLeg2 1 1];
            isStable = stabilityCheck(leg);
            %fprintf("dx_b:%.2f, dy_b: %.2f, sc1(dy_b<): %.2f, sc2(dy_b>):%.2f\n", dx_b, dy_b, stabilityCheckLeg1Phrase, stabilityCheckLeg2Phrase);
        end
        function isStable = check_stability_leg_length(x_a_prime, y_a_prime, z_a_prime, ...
                x_h_prime, y_h_prime, z_h_prime, legLength)
            %% OK for 4 legs
            stepSize = (x_a_prime-x_h_prime)^2 + (y_a_prime-y_h_prime)^2 + (z_a_prime-z_h_prime)^2;
            isStable = stepSize < legLength^2;
            %fprintf("stability leg length - x_a_prime: %.2f, x_h_prime: %.2f, xap-xhp: %.2f\n", x_a_prime, x_h_prime, abs(x_a_prime-x_h_prime)); 
            %fprintf("stability leg length - %.2f <? %.2f\n", stepSize, legLength^2); 
        end
        %% Other Helper Methods
        function pathAngle = check_path_angle_at_future_legs(leg, path)
            [futurePathPoint, futurePathIndex] = bot.get_next_path_point(state, path, moveX);
            %disp("FT PATH PT")
            %disp(futurePathPoint);
            triedPathPts(c,:) = futurePathPoint;
            [angle, slope] = path.get_gamma_at_index(futurePathIndex);
        end
        
    end
end