%{
Follows along path and gets farthest swing leg position that is valid
Uses pose to get leg end positions
%}
classdef SwingPose < SwingStrategy
    properties(Constant)
        strategyName = "SwingPose";
        setDesiredPose = false;
    end
    properties
        desiredPose;
    end
    methods
        function obj = SwingPose()
            obj@SwingStrategy;
        end
        function globalFootPoint = get_next_global_foot_point(obj, bot, state, leg, terrain, path, moveX)
            [futurePathPoint, futurePathIndex] = bot.get_next_path_point(state, path, moveX);
            %fprintf("path index: %d\n", futurePathIndex);
            %fprintf("base pos: %.2f %.2f\n", state.basePosition(1:2));
            [angle, slope] = path.get_gamma_at_index(futurePathIndex);
            %disp("getting next path pt")
            %fprintf("current base pos: %.2f %.2f %.2f, currentGamma: %.2f, moveX: %.2f, ft path pt: %.2f %.2f\n", state.basePosition, ...
            %    state.baseOrientation(3), moveX, futurePathPoint);
            futureState = BotState(state);
            futureState.basePosition(1:2) = futurePathPoint(1:2);
            futureState.baseOrientation(3) = angle;
            globalFootPoint = bot.get_global_end_positions(futureState, leg);
            %disp('base center point');
            %disp(futurePathPoint);
            %globalFootPoint = footPoints(leg,:);
        end
    end
end