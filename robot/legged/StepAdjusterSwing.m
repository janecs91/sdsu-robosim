classdef StepAdjusterSwing < StepAdjuster
    properties
        % limit ranges
        minDxi = 0;
        frontCornerDistance;
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
        modifyDx_i = true;

        verbose = true;
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
            if nargin < 2
                strategy = 1;
            end
            fprintf("SAFETY VALUE SWING: %d", safetyValue)
            obj@StepAdjuster(safetyValue);
            obj.frontCornerDistance = sqrt(2)*15.5;     % How was this calculated?
            %obj.backCornerDistance = sqrt(2)*15.5;      % Is this correct?
            %obj.minDxi = obj.frontCornerDistance;
            obj.minDxi = 0;
            
            strategies = {SwingParallel() SwingPose()};
            obj.strategy = strategies{strategy};
        end
        function [mu_dx_i, mu_dy_i, mu_dz_i] = get_step_vector2(obj, bot, state, leg, terrain, path)
            %% Step vector requirements is different for legs 1 and 2 (front) vs legs 3 and 4 (back)
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
            z_h = jointPositions(1,3);
            x_h = jointPositions(1,1);
            y_h = jointPositions(1,2);
            distance_waist_to_foot = bot.get_distance_waist_to_foot(state, leg);
            %% Possibly update to backCornerDistance for back legs (if trying for legs 3 and 4)
            % Possibly use 1/2 leg length ??
            %dx_i_front = (obj.maxLegLength-distance_waist_to_foot)+obj.frontCornerDistance;
            %dx_i_front = obj.maxLegLength;
            dx_i_front = 68;
            dx_i_back = distance_waist_to_foot;
            %dx_i = obj.maxLegLength + obj.frontCornerDistance;
            %dx_i = obj.maxLegLength;
            dx_i = dx_i_front;
            if leg > 2
                dx_i = dx_i_back;
            end
            dz_i = 0;
            testRange = dx_i:-1:obj.minDxi;
            if obj.verbose && false
                fprintf("find swing step: leg %d - x_a: %.2f, y_a: %.2f, z_a: %.2f\n", leg, x_a, y_a, z_a);
                fprintf("x_h: %.2f, y_h: %.2f, z_h: %.2f\n", x_h, y_h, z_h);
                fprintf("local waist x: %.2f, distance_waist_to_foot: %.2f, maxdx_i: %.2f\n", waist_x, distance_waist_to_foot, dx_i);
                disp("testRange")
                disp(testRange);
            end
            for i=testRange
                %fprintf("* test range i: %d\n", i);
                moveX = i;
                globalFootPoint = obj.strategy.get_next_global_foot_point(bot, state, leg, terrain, path, moveX);
                newTerrainZ = terrain.get_elevation(globalFootPoint(1), globalFootPoint(2));
                globalFootPoint(3) = newTerrainZ;
                localFootPoint = bot.change_global_to_local(state, globalFootPoint);
                localOldFootPoint = bot.change_global_to_local(state, state.endPositions(leg,1:2));
                if obj.verbose
                    fprintf("state pos: %.2f %.2f, gamma: %.2f\n", state.basePosition(1:2), rad2deg(state.baseOrientation(3)));
                    fprintf("(global) old ft pt: %.2f %.2f %.2f, new ft pt: %.2f %.2f %.2f\n", ...
                        state.endPositions(leg,:), globalFootPoint);
                    fprintf("(local) old ft pt: %.2f %.2f, local old2: %.2f %.2f %.2f, new ft pt: %.2f %.2f %.2f\n", ...
                        [x_a y_a], localOldFootPoint, localFootPoint);
                end
                
                x_a_prime = localFootPoint(1);
                y_a_prime = localFootPoint(2);
                z_a_prime = localFootPoint(3);
                [x_h, y_h, z_h, x_h_prime, y_h_prime, z_h_prime] = StepAdjusterSwing.get_hips(bot, state, leg, x_a_prime, y_a_prime, z_a_prime);    % for display only
                if obj.verbose && false
                    fprintf("trying leg %d swing step - moveX: %.2f\n", leg, moveX)
                    fprintf("x_a_prime: %.2f, y_a_prime: %.2f, z_a_prime: %.2f\n", x_a_prime, y_a_prime, z_a_prime);
                    fprintf("x_h_prime: %.2f, y_h_prime: %.2f, z_h_prime: %.2f\n", x_h_prime, y_h_prime, z_h_prime);
                end
                isStable = StepAdjusterSwing.check_stability_swing(bot, state, leg, x_a, y_a, z_a, ...
                    x_a_prime, y_a_prime, z_a_prime, obj.maxLegLength);
                %fprintf("stable? %d\n", isStable);
                if isStable
                    if obj.verbose
                        %fprintf("found stable leg %d step, moveX: %d", leg, moveX)
                        fprintf("x_a: %.2f, y_a: %.2f, z_a: %.2f\n", x_a, y_a, z_a);
                        fprintf("x_h: %.2f, y_h: %.2f, z_h: %.2f\n", x_h, y_h, z_h);
                        fprintf("x_a_prime: %.2f, y_a_prime: %.2f, z_a_prime: %.2f\n", x_a_prime, y_a_prime, z_a_prime);
                        fprintf("x_h_prime: %.2f, y_h_prime: %.2f, z_h_prime: %.2f\n", x_h_prime, y_h_prime, z_h_prime);
                        %fprintf("x_a_prime: %.2f, x_h_prime: %.2f, xap-xhp: %.2f\n", x_a_prime, x_h_prime, abs(x_a_prime-x_h_prime)); 
                        %fprintf("y_a_prime: %.2f, y_h_prime: %.2f, yap-yhp: %.2f\n", y_a_prime, y_h_prime, abs(y_a_prime-y_h_prime)); 
                        %fprintf("z_a_prime: %.2f, z_h_prime: %.2f, zap-zhp: %.2f\n", z_a_prime, z_h_prime, abs(z_a_prime-z_h_prime)); 
                    end
                    break
                else
                    if obj.verbose && false
                        fprintf("NOT stable --> after trying leg %d swing step - moveX: %.2f\n", leg, moveX)
                    end
                end
            end
            disp(moveX);
            dx_i = 24;
            dy_i = 0;
            if obj.modifyDx_i
                dx_i = x_a_prime-x_a;
                dy_i = y_a_prime-y_a;
            end
            if obj.verbose
                fprintf("stable moveX for leg %d: %.2f\n", leg, moveX);
                fprintf("dx_i: %.2f, dy_i: %.2f\n", dx_i, dy_i);
            end
            mu_dx_i = max(obj.safetyValue*dx_i, 0);
            if obj.allowNegativeDxi == true
                mu_dx_i = dx_i;
            end
            %mu_dy_i = dy_i;
            mu_dy_i = obj.safetyValue*dy_i;
            mu_dz_i = dz_i;
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