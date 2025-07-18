classdef WalkKinematics
    properties
        epsilon_small_value = 0.0000001;
        useTurnAngleValue = false;
        useMockValuesStance = false;
        useMockValuesSwing = false;
        useAltWaist = 2;
        mockTurnAngle = deg2rad(5);
        mock_dx_b = 20;
        mock_dx_i = 10;
        verbose = false;
        swingVerbose = false;
    end
    properties(Constant)
    end
    methods
        function obj = WalkKinematics()
        end
        %% Eval Methods
        function [dot_waist, dot_hip, dot_knee, waist, hip, knee] = eval_alt_kinematics_swing2(obj, bot, inputState)
            % This is for only the first 2 legs
            % pull only - cannot handle 4 legs?
            % sigma = slope of path
            % slope
            % not implemented?
            state = copy(inputState);
            activeLeg = state.activeLeg;    % CAN THIS WORK WITH 4 LEGS?
            altLeg = mod(activeLeg,2)+1;    % IS THIS CORRECT?
            endPos = bot.get_global_end_positions(state);
            globalJointPos = bot.get_global_joint_positions(state, activeLeg);
            jointPositions = bot.get_local_joint_positions(state, activeLeg);
            jointPositions = jointPositions{1};
            x_a = jointPositions(end,1);
            y_a = jointPositions(end,2);
            z_a = jointPositions(end,3);
            x_h = jointPositions(3,1);
            y_h = jointPositions(3,2);
            z_h = jointPositions(3,3);
            dx_i = state.dotEndPositions(activeLeg,1);
            dy_i = state.dotEndPositions(activeLeg,2);
            dz_i = state.dotEndPositions(activeLeg,3);
            if obj.useMockValuesSwing
                %dx_i = obj.mock_dx_i;
                dx_i = 20;
                dy_i = 20;
                dz_i = 0;
            end
            %{
            disp("Walk Kin Input+++++++++++++++++++++");
            fprintf("dx_i: %.2f, dy_i: %.2f, dz_i: %.2f\n", dx_i, dy_i, dz_i);
            %}
            slopeLeg = 0;
            if dx_i ~= 0
                slopeLeg = dy_i/dx_i;
                %slopeLeg = atan2(dy_i,dx_i);
            end
            % transform
            x_a_prime = x_a + dx_i;
            y_a_prime = y_a + dy_i;
            z_a_prime = z_a + dz_i;
            %{
            numerator = [x_a_prime-bot.a_0 y_a_prime-bot.a_0];     %correct
            denominator = [-y_a_prime-bot.a_0 x_a_prime-bot.a_0];    %correct
            hat_waist = atan2(numerator(activeLeg), denominator(activeLeg));
            waist = hat_waist-deg2rad(45);
            %}
            gamma = 0;
            [waist, hat_waist] = WalkKinematics.get_waist_given_prime(bot, activeLeg, x_a_prime, y_a_prime, gamma);
            
            %% get new hip locations
            [x_h_prime, y_h_prime, z_h_prime] = WalkKinematics.get_hip_location_given_waist(bot, state, waist, activeLeg);
            
            
            %% get A,B,C
            x_diff = x_a_prime-x_h_prime;
            y_diff = y_a_prime-y_h_prime;
            z_diff = z_a_prime-z_h_prime;
            A = 2*bot.a_4*(z_diff);
            divSign = [-1 1];
            divParts1 = [x_diff y_diff];
            divParts2 = [sin(hat_waist) divSign(activeLeg)*cos(hat_waist)];
            diff_by_w =  divParts1(1)/divParts2(activeLeg);
            legRangesMin = [-1 89];
            legRangesMax = [1 91];
            B_sign = [1 1];
            if abs(hat_waist) > deg2rad(legRangesMin(activeLeg)) && abs(hat_waist) < deg2rad(legRangesMax(activeLeg))
                diff_by_w = divParts1(2)/divParts2(altLeg);
                %B_sign = [-1 1];
            end
            B = B_sign(activeLeg)*2*bot.a_4*diff_by_w;
            C = -diff_by_w^2 - (z_diff)^2;
            
            %{
            disp("A, B, C Calculations ====");
            fprintf("x_a_prime: %.2f \ny_a_prime: %.2f \nz_a_prime: %.2f \nx_diff: %.2f \nz_diff: %.2f \n",x_a_prime, y_a_prime, z_a_prime, x_diff, z_diff);
            fprintf("A: %.2f \nA^2: %.2f \nB: %.2f \nB^2: %.2f \nC: %.2f \nC^2: %.2f \n", A, A^2, B, B^2, C, C^2);
            fprintf("A^2+B^2: %.2f \nA^2+B^2-C^2: %.2f\n",A^2+B^2, A^2+B^2-C^2);
            %}
            
            if A^2+B^2-C^2 < 0
                fprintf("dx_i: %.2f, dy_i: %.2f, dz_i: %.2f\n", dx_i, dy_i, dz_i);
                error(sprintf("Delta part becomes imaginary number for Leg %d", activeLeg));
            end
            
            %% get delta
            delta_part1 = atan2(B,A);
            delta_part2 = atan2(sqrt(A^2+B^2-C^2),C);
            
            % multiple joint solutions, choose best fit
            deltas = [delta_part1+delta_part2 delta_part1-delta_part2];
            hips = [acos((bot.a_4*cos(deltas(1))+z_diff)/bot.a_3) acos((bot.a_4*cos(deltas(2))+z_diff)/bot.a_3)];
            knees = [hips(1)-deltas(1) hips(2)-deltas(2)];
                        
            delta_i = 2;
            hip = hips(delta_i);
            knee = knees(delta_i);
            
            % get dots
            dot_waist = waist - state.anglesWaist(activeLeg);
            dot_hip = hip - state.anglesHip(activeLeg);
            dot_knee = knee - state.anglesKnee(activeLeg);
                
            % check good delta
            oldEndPosition = endPos(activeLeg,:);
            newEndPositions = zeros(2,3);
            newGamma = 0;
            [newEndPosition1, newState1] = WalkKinematics.check_new_end_position(bot, state, activeLeg, 0, 0, waist, hips(1), knees(1), newGamma);
            [newEndPosition2, newState2] = WalkKinematics.check_new_end_position(bot, state, activeLeg, 0, 0, waist, hips(2), knees(2), newGamma);
            futureLocalEndPosition = bot.get_local_end_positions(newState2, activeLeg);
            %localEndPositions = bot.global_to_local(state, activeLeg);
                        
            %% debug
            %{
            fprintf("SWING JOINTS LEG %d =========", activeLeg);
            
            %% debug
            disp("Initial Joint Results========")
            fprintf("waist: %.2f \nhat_waist: %.2f \nhip: %.2f \nknee: %.2f\n", ...
                rad2deg(state.anglesWaist(activeLeg)), rad2deg(state.anglesWaist(activeLeg))-45, ...
                rad2deg(state.anglesHip(activeLeg)), rad2deg(state.anglesKnee(activeLeg)));
            disp("WAIST Calculations ========")
            fprintf("slopeLeg: %.2f \nx_a: %.2f \ny_a: %.2f \nz_a: %.2f\n", slopeLeg, x_a, y_a, z_a);
            fprintf("x_h: %.2f \ny_h: %.2f \nz_h: %.2f\n", x_h, y_h, z_h);
            fprintf("dx_i: %.2f \nslope*dx_i: %.2f \ndy_i: %.2f \ndz_i: %.2f\n",dx_i, slopeLeg*dx_i, dy_i, dz_i);
            fprintf('hat waists: %.2f %.2f chosen %.2f\n', rad2deg(hat_waist), rad2deg(hat_waist), rad2deg(hat_waist));
            fprintf('new_waists: %.2f %.2f chosen %.2f\n', rad2deg(waist), rad2deg(waist), rad2deg(waist));
            
            disp("Joint Results========")
            fprintf("deltas: %.2f %.2f\n", rad2deg(deltas));
            fprintf("chosen delta index: %d\n", delta_i);
            fprintf("old_waist: %.2f \nold_hip: %.2f \nold_knee: %.2f\n", rad2deg(state.anglesWaist(activeLeg)), rad2deg(state.anglesHip(activeLeg)), rad2deg(state.anglesKnee(activeLeg)));
            fprintf("new_waist1: %.2f \nnew_hip1: %.2f \nnew_knee1: %.2f\n", rad2deg(waist), rad2deg(hips(1)), rad2deg(knees(1)));
            fprintf("new_waist2: %.2f \nnew_hip2: %.2f \nnew_knee2: %.2f\n", rad2deg(waist), rad2deg(hips(2)), rad2deg(knees(2)));
            fprintf("dot_waist: %.2f \ndot_hip: %.2f \ndot_knee: %.2f\n", rad2deg(dot_waist), rad2deg(dot_hip), rad2deg(dot_knee));
            disp(" ");
            %}
            if obj.verbose == true || obj.swingVerbose == true
                fprintf("IS END POSITION SAME? SWING LEG %d====================\n",activeLeg);
                fprintf("endPos %.5f %.5f %.5f \n", oldEndPosition);
                fprintf("newEndPositions(1) %.5f %.5f %.5f \n", newEndPosition1);
                fprintf("newEndPositions(2) %.5f %.5f %.5f \n\n", newEndPosition2);
                fprintf("local desired: %.2f %.2f, local new: %.2f %.2f\n", [x_a_prime, y_a_prime], futureLocalEndPosition(1:2));
            end
            
            %BotPlotter.plot_triangle(bot, state, newState, x_a, y_a, 0, 0, 0, 0, turnAngle, activeLeg);
            
        end
        function [dot_waist, dot_hip, dot_knee] = eval_alt_kinematics_stance(obj, bot, state)
            % pull only - cannot handle 4 legs?
            % sigma = slope of path
            % slope
            activeLeg = state.activeLeg;
            altLeg = mod(activeLeg,2)+1;
            oldBasePosition = state.basePosition;
            endPos = bot.get_global_end_positions(state);
            jointPositions = bot.get_local_joint_positions(state, altLeg);
            jointPositions = jointPositions{1};
            x_a = jointPositions(end,1);
            y_a = jointPositions(end,2);
            z_a = jointPositions(end,3);
            x_h = jointPositions(3,1);
            y_h = jointPositions(3,2);
            z_h = jointPositions(3,3);
            
            %% remove
            turnAngle = state.dotBaseOrientation(3);
            %slope = tan(turnAngle);
            dx_b = state.dotBasePosition(1);
            dy_b = state.dotBasePosition(2);
            dz_b = state.dotBasePosition(3);
            if obj.useMockValuesStance
                dx_b = obj.mock_dx_b;
                dy_b = 0;
            end
            if obj.verbose == true
                disp("Walk Kin Inputs+++++++++++++++++++++");
                fprintf("dx_b: %.2f, dy_b: %.2f\n", dx_b, dy_b);
            end
            
            %% keep
            a=bot.a_0+(sqrt(2)/2)*bot.a_2;
            
            %% waist
            %{
            x_a_part = x_a-dx_bw-bot.a_0;
            y_a_parts = [y_a-dy_b+bot.a_0 y_a-dy_bw-bot.a_0];
            if x_a_part == 0
                x_a_part = x_a_part + obj.epsilon_small_value;
            end
            waist_numer = [x_a_part y_a_parts(2)];
            waist_denom = [-y_a_parts(1) x_a_part];
            hat_waist = atan2(waist_numer(altLeg), waist_denom(altLeg));
            hat_waist = hat_waist-gamma;
            waist = hat_waist-deg2rad(45);
            %}
            
            gamma = turnAngle;
            [x_a_prime, y_a_prime] = WalkKinematics.get_xy_primes_given_vector_and_gamma(bot, state, altLeg, dx_b, dy_b, gamma);
            [x_a_prime0, y_a_prime0] = bot.get_new_location(x_a, y_a, -dx_b, -dy_b, gamma);
            if obj.verbose == true
                fprintf("****checkWK Stance--\n x_a_prime: %.2f, y_a_prime: %.2f \n x_a_prime0: %.2f, y_a_prime0: %.2f\n", x_a_prime, y_a_prime, x_a_prime0, y_a_prime0);
                fprintf("prime diff: %.2f %.2f\n", x_a_prime-x_a_prime0, y_a_prime-y_a_prime0);
            end
            z_a_prime = z_a-dz_b;
            [waist, hat_waist] = obj.get_waist_given_prime2(bot, altLeg, x_a_prime, y_a_prime, gamma);
            if obj.verbose == true
                fprintf("wk - gamma: %.2f, old_waist: %.2f, hat_waist: %.2f, waist: %.2f\n", rad2deg(gamma), rad2deg(state.anglesWaist(altLeg)), rad2deg(hat_waist), rad2deg(waist));
            end
            %fprintf("kin stance move leg %d - old waist: %.2f, new waist: %.2f\n", altLeg, rad2deg(state.anglesWaist(altLeg)), rad2deg(waist));
            
            %{
            numerator = [x_a_prime-bot.a_0 y_a_prime-bot.a_0];     %correct
            denominator = [-y_a_prime-bot.a_0 x_a_prime-bot.a_0];    %correct
            hat_waist = atan2(numerator(activeLeg), denominator(activeLeg));
            waist = hat_waist-deg2rad(45);
            %}
            %waist = WalkKinematics.get_waist_given_prime(bot, activeLeg, x_a_prime, y_a_prime, z_a_prime);
            
            %{
            x_a_part = x_a-dx_bw-bot.a_0;
            x_a_part = x_a_prime-bot.a_0;
            y_a_parts = [y_a-dy_b+bot.a_0 y_a-dy_bw-bot.a_0];
            y_a_parts = [y_a_prime+bot.a_0 y_a_prime-bot.a_0];
            %}
            %B_sin_parts = [x_a_part y_a_parts(2)];
            %B_cos_parts = [y_a_parts(1) x_a_part];
            B_sin_parts = [x_a_prime-bot.a_0 y_a_prime-bot.a_0];
            B_cos_parts = [y_a_prime+bot.a_0 x_a_prime-bot.a_0];
            B_signs = [-1 1];
            B_sin = (1/sin(hat_waist))*B_sin_parts(altLeg)-bot.a_2;
            B_cos = B_signs(altLeg)*(1/cos(hat_waist))*B_cos_parts(altLeg)-bot.a_2;
            
            %% get A,B,C
            legRangesMin = [-1 89];
            legRangesMax = [1 91];
            if abs(hat_waist) > deg2rad(legRangesMin(altLeg)) && abs(hat_waist) < deg2rad(legRangesMax(altLeg))
                Bs = [B_cos B_sin];
                B = Bs(altLeg);
            else
                Bs = [B_sin B_cos];
                B = Bs(altLeg);
            end
            abs(hat_waist)
            A = z_a - dz_b + bot.d_1;
            C=-A^2-B^2;
            
            %{
            disp("A, B, C Calculations ====");
            fprintf("A: %.2f \nA^2: %.2f \nB: %.2f \nB^2: %.2f \nC: %.2f \nC^2: %.2f \n", A, A^2, B, B^2, C, C^2);
            fprintf("A^2+B^2: %.2f \nA^2+B^2-C^2: %.2f\n",A^2+B^2, A^2+B^2-C^2);
            %}
            
            %% get delta
            delta_part1 = deg2rad(90);
            delta_part2 = deg2rad(90);
            delta_part1_sign=[1 1];
            if A~=0
                delta_part1=atan2(delta_part1_sign(altLeg)*B, A);
            end
            if C~=0
                delta_part2_phrase = (2*A*bot.a_4)^2+(2*B*bot.a_4)^2-C^2;
                if delta_part2_phrase < 0
                    fprintf("dx_b: %.2f, dy_b: %.2f, dz_b: %.2f\n", dx_b, dy_b, dz_b);
                    error(sprintf("Delta part becomes imaginary number for Leg %d", altLeg));
                end
                delta_part2=atan2(sqrt(delta_part2_phrase),C);
            end
                        
            % multiple joint solutions, choose best fit
            deltas = [delta_part1+delta_part2 delta_part1-delta_part2];
            %deltas(2) = deltas(3);
            hips = [acos((bot.a_4*cos(deltas(1))+A)/bot.a_3) acos((bot.a_4*cos(deltas(2))+A)/bot.a_3)];
            knees = [hips(1)-deltas(1) hips(2)-deltas(2)];
            
            % check good delta
            %newGamma = state.baseOrientation(3);
            gamma = turnAngle;
            newGamma = gamma;
            globalMoveVector = bot.rotate_vector([dx_b dy_b], newGamma);
            newBasePosition(1:2) = state.basePosition(1:2) + globalMoveVector;
            oldEndPosition = endPos(altLeg,:);
            newEndPositions = zeros(2,3);
            [newEndPosition1, newState1] = WalkKinematics.check_new_end_position(bot, state, altLeg, dx_b, dy_b, waist, hips(1), knees(1), newGamma);
            [newEndPosition2, newState2] = WalkKinematics.check_new_end_position(bot, state, altLeg, dx_b, dy_b, waist, hips(2), knees(2), newGamma);
            newEndPositions(1,:) = newEndPosition1;
            newEndPositions(2,:) = newEndPosition2;
            futureLocalEndPosition = bot.get_local_end_positions(newState2, altLeg);
            [errorDistance, errorAngle] = bot.get_vector_to_point(oldEndPosition(1:2), newEndPositions(2,1:2));
            [oldBaseToOldDistance, oldBaseToOldAngle] = bot.get_vector_to_point(oldBasePosition(1:2),oldEndPosition(1:2));
            [newBaseToOldDistance, newBaseToOldAngle] = bot.get_vector_to_point(newBasePosition(1:2),oldEndPosition(1:2));
            [newBaseToNewDistance, newBaseToNewAngle] = bot.get_vector_to_point(newBasePosition(1:2),newEndPositions(2,1:2));
            localBaseToOldAngle = oldBaseToOldAngle - (state.baseOrientation(3));
            localBaseToNewAngle = newBaseToNewAngle - (state.baseOrientation(3)+gamma);
            
            delta_i = 2;
            hip = hips(delta_i);
            knee = knees(delta_i);
            
            % get dots
            dot_waist = waist - state.anglesWaist(altLeg);
            dot_hip = hip - state.anglesHip(altLeg);
            dot_knee = knee - state.anglesKnee(altLeg);
            
            %% debug: check result
            
            %% debug
            %{
            disp("Initial Joint Results========")
            fprintf("waist: %.2f \nhat_waist: %.2f \nhip: %.2f \nknee: %.2f\n", ...
                rad2deg(state.anglesWaist(altLeg)), rad2deg(state.anglesWaist(altLeg))-45, ...
                rad2deg(state.anglesHip(altLeg)), rad2deg(state.anglesKnee(altLeg)));
            disp("WAIST Calculations ========")
            fprintf("a_0: %.2f\nslope: %.2f \n", bot.a_0, slope);
            fprintf("x_a: %.2f \ny_a: %.2f \nz_a: %.2f\n", x_a, y_a, z_a);
            fprintf("x_h: %.2f \ny_h: %.2f \nz_h: %.2f\n", x_h, y_h, z_h);
            fprintf("dx_b: %.2f \nslope*dx_b: %.2f \ndy_b: %.2f \n",dx_b, slope*dx_b, dy_b);
            fprintf('new_hat_waist: %.2f\nnew_waist: %.2f \n', rad2deg(hat_waist), rad2deg(waist));
            
            disp('B parts ====');
            fprintf(">> cos(new_hat_waist) %.2f 1/cos(new_hat_waist) %.2f\n", cos(hat_waist), 1/cos(hat_waist));
            fprintf(">> sin(new_hat_waist) %.2f 1/sin(new_hat_waist) %.2f\n", sin(hat_waist), 1/sin(hat_waist));
            
            disp("Joint Results========")
            fprintf("deltas: %.2f %.2f\n", rad2deg(deltas));
            fprintf("old_waist: %.2f \nold_hip: %.2f \nold_knee: %.2f\n", rad2deg(state.anglesWaist(altLeg)), rad2deg(state.anglesHip(altLeg)), rad2deg(state.anglesKnee(altLeg)));
            fprintf("new_waist1: %.2f \nnew_hip1: %.2f \nnew_knee1: %.2f\n", rad2deg(waist), rad2deg(hips(1)), rad2deg(knees(1)));
            fprintf("new_waist2: %.2f \nnew_hip2: %.2f \nnew_knee2: %.2f\n", rad2deg(waist), rad2deg(hips(2)), rad2deg(knees(2)));
            fprintf("dot_waist: %.2f \ndot_hip: %.2f \ndot_knee: %.2f\n", rad2deg(dot_waist), rad2deg(dot_hip), rad2deg(dot_knee));
            disp(" "); 
            %}
            if obj.verbose == true && dx_b ~= 0
                fprintf("IS END POSITION SAME? STANCE LEG %d====================\n",altLeg);
                fprintf("hat_waist: %.2f, old_waist: %.2f, new waist: %.2f\n", rad2deg(hat_waist), rad2deg(state.anglesWaist(altLeg)), rad2deg(waist));
                fprintf("dx: %.2f, dy: %.2f, gamma: %.2f\n", dx_b, dy_b, rad2deg(newGamma));
                fprintf("globalMoveVector: %.2f %.2f\n", globalMoveVector(1:2));
                fprintf("old base position %.2f %.2f %.2f orient %.2f\n", oldBasePosition, rad2deg(state.baseOrientation(3)));
                fprintf("new base position %.2f %.2f orient %.2f\n", newBasePosition(1:2), rad2deg(state.baseOrientation(3)+newGamma));
                fprintf("old endPosition %.5f %.5f %.5f \n", oldEndPosition);
                fprintf("chosen delta index: %d\n", delta_i);
                fprintf("newEndPositions(1) %.5f %.5f %.5f \n", newEndPositions(1,:));
                fprintf("newEndPositions(2) %.5f %.5f %.5f \n", newEndPositions(2,:));
                fprintf("ERROR (diff): %.5f %.5f %.5f || distance %.5f angle %.5f \n", newEndPositions(2,:)-oldEndPosition, errorDistance, rad2deg(errorAngle));
                fprintf("newLocalEndPositions(2) %.5f %.5f %.5f \n", futureLocalEndPosition);
                fprintf("desiredLocalEnd: %.5f %.5f\n", [x_a_prime, y_a_prime]);
                fprintf("oldBaseToOldDistance: %.5f, oldBaseToOldAngle: %.5f localBaseToOldAngle: %.5f\n", oldBaseToOldDistance, rad2deg(oldBaseToOldAngle), rad2deg(localBaseToOldAngle));
                fprintf("newBaseToOldDistance: %.5f, newBaseToOldAngle: %.5f\n", newBaseToOldDistance, rad2deg(newBaseToOldAngle));
                fprintf("newBaseToNewDistance: %.5f, newBaseToNewAngle: %.5f localBaseToNewAngle: %.5f\n", newBaseToNewDistance, rad2deg(newBaseToNewAngle), rad2deg(localBaseToNewAngle));
                fprintf("angleDiff: %.5f\n", rad2deg(newBaseToNewAngle-newBaseToOldAngle));
                fprintf("\n")
            end
            if oldEndPosition ~= newEndPositions(2,:)
                disp("TROUBLE");
            end
            
            %BotPlotter.plot_triangle(bot, state, newState, x_a, y_a, dx_b, dy_b, 0, 0, turnAngle, altLeg);
        end
        
    end
    methods(Static)
        %% Eval Methods
        function [A, By] = eval_leg_function(bot, state)
            leg = state.activeLeg;
            matrix_function = bot.functionsAByLeg{leg};
            [A, By] = matrix_function(state.anglesWaist(leg), state.anglesHip(leg), state.anglesKnee(leg), ...
                state.dotEndPositions(leg,:), state.dotEndOrientations(leg,:), ...
                state.dotBasePosition, state.dotBaseOrientation); 
        end
        function [A, By] =  eval_body_function(bot, state)
            matrix_function = bot.functionsAByBody{state.activeLeg};
            [A, By] = matrix_function(state.anglesWaist, state.anglesHip, state.anglesKnee, ...
                state.dotEndPositions, state.dotEndOrientations, ...
                state.dotBasePosition, state.dotBaseOrientation);
        end
        function [dot_waist, dot_hip, dot_knee] = eval_kinematics_swing(bot, state)
            [A_eval, By_eval] = WalkKinematics.eval_leg_function(bot, state);
            % inverse A
            pA_eval = pinv(A_eval);
            knownVector = pA_eval*By_eval;
            
            waistChange = knownVector(1,1);
            hipChange = knownVector(2,1);
            kneeChange = knownVector(3,1);
            
            % update joints
            dot_waist = waistChange;
            dot_hip = hipChange;
            dot_knee = kneeChange;
        end
        function [dot_waist, dot_hip, dot_knee] = eval_kinematics_stance(bot, state)
            [A_eval, By_eval] = WalkKinematics.eval_body_function(bot, state);
            % inverse A
            pA_eval = pinv(A_eval);
            knownVector = pA_eval*By_eval;
            
            % update joints
            if bot.numLegs == 4
                % walk
                waistChange = knownVector(1:3,1);
                hipChange = knownVector(4:6,1);
                kneeChange = knownVector(7:9,1);
            else
                % pull
                waistChange = knownVector(1,1);
                hipChange = knownVector(2,1);
                kneeChange = knownVector(3,1);
            end
            
            % update joints
            dot_waist = waistChange;
            dot_hip = hipChange;
            dot_knee = kneeChange;
        end
        function [new_waist, waist_hat] = get_waist_given_prime(bot, leg, x_a_prime, y_a_prime, gamma)
            % LEGS < 4 !!!!
            %{
            numerator = [x_a_prime-bot.a_0 y_a_prime-bot.a_0];     %correct
            denominator = [-y_a_prime-bot.a_0 x_a_prime-bot.a_0];    %correct
            hat_waist = atan2(numerator(activeLeg), denominator(activeLeg));
            waist = hat_waist-deg2rad(45);
            %}
            %leg = mod(leg,2)+1;
            % TBD: FIX
            numerator = [x_a_prime-bot.a_0 y_a_prime-bot.a_0 y_a_prime-bot.a_0 x_a_prime-bot.a_0];
            denominator = [-y_a_prime-bot.a_0 x_a_prime-bot.a_0 x_a_prime-bot.a_0 -y_a_prime-bot.a_0];
            waist_hat = atan2(numerator(leg), denominator(leg));
            %fprintf("waist calc: given gamma %.2f\n", rad2deg(gamma));
            %waist_hat = waist_hat - gamma;
            new_waist = waist_hat - deg2rad(45);
        end
        function [new_waist, waist_hat] = get_waist_given_prime2(bot, leg, x_a_prime, y_a_prime, gamma)
            % This is for the alt kinematics 2 (front 2 legs)
            % LEGS < 4 !!!!
            %{
            numerator = [x_a_prime-bot.a_0 y_a_prime-bot.a_0];     %correct
            denominator = [-y_a_prime-bot.a_0 x_a_prime-bot.a_0];    %correct
            hat_waist = atan2(numerator(activeLeg), denominator(activeLeg));
            waist = hat_waist-deg2rad(45);
            %}
            numerator = [x_a_prime-bot.a_0 y_a_prime-bot.a_0];
            denominator = [-(y_a_prime+bot.a_0) x_a_prime-bot.a_0];
            waist_hat = atan2(numerator(leg), denominator(leg));
            %fprintf("numerator: %.2f, denominator: %.2f, num/den: %.5f\n", numerator(leg), denominator(leg), numerator(leg)/denominator(leg))
            %fprintf("waist calc: given gamma %.2f\n", rad2deg(gamma));
            %waist_hat = waist_hat - gamma;
            new_waist = waist_hat - deg2rad(45);
        end
        function [x_h, y_h, z_h] = get_hip_location_given_waist(bot, state, waist, leg)
            newState = copy(state);
            newState.anglesWaist(leg) = waist;
            jointPositions = bot.get_local_joint_positions(newState, leg);
            jointPositions = jointPositions{1};
            x_h = jointPositions(3,1);
            y_h = jointPositions(3,2);
            z_h = jointPositions(3,3);
        end
        function [A, B, C, allowed] = get_ABC_swing(bot, activeLeg, altLeg, x_a_prime, y_a_prime, z_a_prime)
            % LEGS < 4 !!!!
            numerator = [x_a_prime-bot.a_0 y_a_prime-bot.a_0];     %correct
            denominator = [-y_a_prime-bot.a_0 x_a_prime-bot.a_0];    %correct
            hat_waist = atan2(numerator(activeLeg), denominator(activeLeg));
            waist = hat_waist-deg2rad(45);
            
            %% get new hip locations
            [x_h_prime, y_h_prime, z_h_prime] = WalkKinematics.get_hip_location_given_waist(bot, state, waist, activeLeg);
            
            %% get A,B,C
            x_diff = x_a_prime-x_h_prime;
            y_diff = y_a_prime-y_h_prime;
            z_diff = z_a_prime-z_h_prime;
            A = 2*bot.a_4*(z_diff);
            divSign = [-1 1];
            divParts1 = [x_diff y_diff];
            divParts2 = [sin(hat_waist) divSign(activeLeg)*cos(hat_waist)];
            diff_by_w =  divParts1(1)/divParts2(activeLeg);
            legRangesMin = [-1 89];
            legRangesMax = [1 91];
            B_sign = [1 1];
            if abs(hat_waist) > deg2rad(legRangesMin(activeLeg)) && abs(hat_waist) < deg2rad(legRangesMax(activeLeg))
                diff_by_w = divParts1(2)/divParts2(altLeg);
                %B_sign = [-1 1];
            end
            B = B_sign(activeLeg)*2*bot.a_4*diff_by_w;
            C = -diff_by_w^2 - (z_diff)^2;
            
            allowed = true;
            if A^2+B^2-C^2 < 0
                allowed = false;
            end
        end
        function [newEndPosition, newState] = check_new_end_position(bot, state, leg, dx_b, dy_b, waist, hip, knee, newGamma)
            newState = copy(state);
            newState.dotBasePosition = [0 0 0];
            newState.dotBasePosition = [dx_b dy_b 0];
            globalMoveVector = bot.rotate_vector(newState.dotBasePosition(1:2), newState.baseOrientation(3));
            globalMoveVector = [globalMoveVector 0];
            newState.basePosition = newState.basePosition + globalMoveVector;
            newState.dotBaseOrientation(3) = newGamma;
            newState.baseOrientation(3) = newState.baseOrientation(3) + newState.dotBaseOrientation(3);
            newState.anglesWaist(leg) = waist;
            newState.anglesHip(leg) = hip;
            newState.anglesKnee(leg) = knee;
            newEndPosition = bot.get_global_end_positions(newState, leg);
            
        end
        function [x_a_prime, y_a_prime] = get_xy_primes_given_vector_and_gamma(bot, state, altLeg, dx_b, dy_b, gamma)
            oldGlobalEndPosition = bot.get_global_end_positions(state, altLeg);
            futureState = bot.get_new_state_given_vector_and_gamma(state, dx_b, dy_b, gamma);
            futureLocalPoint = bot.change_global_to_local(futureState, oldGlobalEndPosition(1:2));
            %fprintf("old base: %.2f %.2f, new base: %.2f %.2f\n", state.basePosition(1:2), futureState.basePosition(1:2));
            %fprintf("oldGlobalEndPosition: %.2f %.2f\n", oldGlobalEndPosition(1:2));
            %fprintf("oldOrientation: %.2f, newOrient: %.2f, gamma: %.2f\n", rad2deg(state.baseOrientation(3)), rad2deg(futureState.baseOrientation(3)), rad2deg(gamma));
            %fprintf("futureLocalPt: %.2f %.2f\n", futureLocalPoint(1:2));
            x_a_prime = futureLocalPoint(1);
            y_a_prime = futureLocalPoint(2);
        end
    end
end