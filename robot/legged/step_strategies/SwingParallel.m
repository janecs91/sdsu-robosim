%{
This gets parallel points for the leg.
Useful if there is a tight curve.
Sometimes the path gamma at body is different from path gamma at leg end.
This balances the leg so that it follows the path gamma closer at leg end.
%}
classdef SwingParallel < SwingStrategy
    properties(Constant)
        strategyName = "SwingParallel";
        uConstant = 5.0;
        legRadius = 48;
        frontCornerDistance = sqrt(2)*15.5;
        useUConstant = true;
        constantLegRadius = false;
        useBaseGamma = true;
        verbose = true;

        constantBackLegGlobalFootPoint = false;
    end
    properties
        SwingPose;
    end
    methods
        function obj = SwingParallel()
            obj@SwingStrategy;
            addpath('robot/legged/step_strategies/');
            obj.SwingPose = SwingPose();
        end
        function globalFootPoint = get_next_global_foot_point(obj, bot, state, leg, terrain, path, moveX)
            if leg > 2
                % custom strategy for back legs
                %[futurePathPoint, futurePathIndex] = bot.get_next_path_point(state, path, moveX);
                %[angle, slope] = path.get_gamma_at_index(futurePathIndex);
                futureState = BotState(state);
                %futureState.basePosition(1:2) = futurePathPoint(1:2);
                %futureState.baseOrientation(3) = angle;
                % get waist x,y
                globalJointPositions = bot.get_global_joint_positions(futureState, leg);
                %disp(globalJointPositions)
                currentEndPosition = globalJointPositions(end, 1:2);
                globalFootPoint = globalJointPositions(3, 1:2);
                if ~obj.constantBackLegGlobalFootPoint
                    [futurePathPoint, futurePathIndex] = path.get_next_nearest_point(currentEndPosition(1), currentEndPosition(2), futureState.pathIndex, moveX+3);
                    halfWidthRobotBody = 15;
                    [plusPoint, minusPoint] = path.get_parallel_point_at_x(futurePathPoint(1), futurePathPoint(2), halfWidthRobotBody, futurePathIndex);
                    footPoints = [minusPoint; plusPoint; plusPoint; minusPoint];
                    globalFootPoint = footPoints(leg,:);
                    if obj.verbose
                        fprintf("planned base position: %.2f %.2f\n", futureState.basePosition(1:2));
                        fprintf("futurePathPoint: %.2f %.2f\n", futurePathPoint);
                        fprintf("moveX: %.2f\n", moveX);
                    end
                end
            else
                % parallel strategy (front legs)
                %{
                [futurePathPoint, endIndex] = bot.get_next_path_point(state, path, 0);
                startIndex = state.pathIndex;
                s = 1;
                if SwingParallel.useUConstant == true
                    s = SwingParallel.get_ratio_by_path_curve(leg, path, startIndex, endIndex);
                end
                %}
                adjustedLegRadius = moveX;
                if obj.constantLegRadius
                    adjustedLegRadius = obj.legRadius/2;
                end
                %fprintf("end index: %d\n", endIndex);
                %fprintf("before movex: %.2f -- s value %.2f -- after movex: %.2f\n", moveX, s, adjustedMoveX);
                fakeState = copy(state);
                %fakeState.pathIndex = endIndex;
                %fakeState.basePosition(1:2) = futurePathPoint(1:2);
                %[futurePathPoint, futurePathIndex] = bot.get_next_path_point(fakeState, path, moveX);
                %[futurePathPoint, futurePathIndex] = bot.get_next_path_point(fakeState, path, obj.legRadius/2);
                %[futurePathPoint, futurePathIndex] = bot.get_next_path_point(fakeState, path, adjustedLegRadius+obj.frontCornerDistance);
                frontCornerDistanceVector = bot.rotate_vector([obj.frontCornerDistance 0], fakeState.baseOrientation(3));
                frontX = fakeState.basePosition(1)+frontCornerDistanceVector(1);
                frontY = fakeState.basePosition(2)+frontCornerDistanceVector(2);
                [futurePathPoint, futurePathIndex] = path.get_next_nearest_point(frontX, frontY, fakeState.pathIndex, adjustedLegRadius);
                if obj.verbose
                    fprintf("planned base position: %.2d %.2d\n", fakeState.basePosition(1:2));
                    fprintf("futurePathPoint: %.2d %.2d\n", futurePathPoint);
                    fprintf("adjustedLegRadius: %.2d\n", adjustedLegRadius);
                end
                %{
                if obj.useBaseGamma
                    futurePathIndex = endIndex;
                end
                %}
                %[angle, slope] = path.get_gamma_at_index(futurePathIndex);
                %fprintf("(swing) current base pos: %.2f %.2f %.2f, currentGamma: %.2f, moveX: %.2f\n", state.basePosition, state.baseOrientation(3), moveX); 
                %fprintf("old ft pt: [%.2f %.2f], ft center path pt: [%.2f %.2f]\n", state.endPositions(leg,1:2), futurePathPoint);
                % plus = left?, minus = right?
                halfWidthRobotBody = bot.a_0;
                %[plusPoint, minusPoint] = path.get_parallel_point_at_index(futurePathIndex, legRadius);
                [plusPoint, minusPoint] = path.get_parallel_point_at_x(futurePathPoint(1), futurePathPoint(2), halfWidthRobotBody, futurePathIndex);
                footPoints = [minusPoint; plusPoint; plusPoint; minusPoint];
                globalFootPoint = footPoints(leg,:);
                %fprintf("parallel pts - plus: %.2f %.2f, minus %.2f %.2f\n", plusPoint, minusPoint);
            end
        end
    end
    methods(Static)
        % Only supports 1 and 2 swing legs ??? (need to check this)
        function s = get_ratio_by_path_curve(leg, path, startIndex, endIndex)
            %disp("leg")
            %disp(leg)
            % for path curves and robot turns
            %leg = mod(leg,2)+1;
            % improvements? = divide into more segments
            %fprintf("ratio indices: %d - %d\n", startIndex, endIndex);
            %[dy2, dy1] = path.get_2nd_derivative_path(startIndex, endIndex);
            [dy2, dy1] = path.get_avg_2nd_derivative_path(startIndex, endIndex);
            %fprintf("dy values for leg %d ******\n dy2: %.2f, dy1: %.2f\n", leg, dy2, dy1);
            dyWithConstant = SwingParallel.uConstant*dy2;
            %fprintf("dyWithConstant: %d\n", dyWithConstant);
            sOptions = [-dyWithConstant dyWithConstant dyWithConstant -dyWithConstant];
            if dy2 > 0
                sOptions = [dyWithConstant -dyWithConstant -dyWithConstant dyWithConstant];
            end
            %fprintf("SwingParallel-s ratio used for leg %d: %.2f\n", leg, sOptions(leg));
            s = 1+sOptions(leg);
            %fprintf("swing triangle result s: %d\n", s);
        end
    end
end