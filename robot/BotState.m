%% BotState
% only use to hold values for robot state
classdef BotState < matlab.mixin.Copyable
    properties
        % path-related
        pathIndex = 1;
        % general
        condA1Leg = 0;
        condA2Leg = 0;
        condA1Body = 0;
        condA2Body = 0;
        % leg in motion
        activeLeg;
        % positions: 
        % row = leg(1-4) or base(5)
        % col = [x y z]
        basePosition = [0 0 0];
        endPositions = zeros(4,3);
        dotBasePosition = [0 0 0];
        dotEndPositions = zeros(4,3);
        plannedEndPositions = zeros(4,3);
        % orientations: row = leg, col = [alpha beta gamma]
        baseOrientation = [0 0 0];
        endOrientations = zeros(4,3);
        dotBaseOrientation = [0 0 0];
        dotEndOrientations = zeros(4,3);
        % joint angles: legs 1-4
        % hip (eta), waist (omega), knee (nu), ankle (epsilon)
        %anglesJoint;
        anglesWaist = [0 0 0 0];
        anglesHip = [0 0 0 0];
        anglesKnee = [0 0 0 0];
        dotAnglesWaist = [0 0 0 0];
        dotAnglesHip = [0 0 0 0];
        dotAnglesKnee = [0 0 0 0];
        % steering (psi): legs 1-4
        anglesSteering = [0 0 0 0];
        dotAnglesSteering = [0 0 0 0];
        % contact (kappa): legs 1-4
        anglesContact = [0 0 0 0];
    end
    properties(Constant)
        %% Save File Properties
        save_path = 'saved/%s%s.mat';
    end
    methods
        function obj = BotState(previousState)
            % load appropriate variables
            if nargin > 0
                obj = copy(previousState);
                obj = clear_dots(obj);
            end
        end
        function obj = clear_dots(obj)
            obj.dotBasePosition = [0 0 0];
            obj.dotEndPositions = zeros(4,3);
            obj.dotBaseOrientation = [0 0 0];
            obj.dotEndOrientations = zeros(4,3);
            obj.dotAnglesWaist = [0 0 0 0];
            obj.dotAnglesHip = [0 0 0 0];
            obj.dotAnglesKnee = [0 0 0 0];
            obj.dotAnglesSteering = [0 0 0 0];
        end
        function matrix = to_vector(obj)
            matrix = [obj.to_vector_base() obj.to_vector_dot_end_positions_flatten() obj.to_vector_dot_joints()];
        end
        function matrix = to_vector_base(obj)
            matrix = [obj.basePosition obj.baseOrientation];
        end
        function matrix = to_vector_end_positions(obj)
            matrix = [obj.endPositions];
        end
        function matrix = to_vector_joints(obj)
            matrix = [obj.anglesWaist obj.anglesHip obj.anglesKnee];
        end
        function matrix = to_vector_dot_joints(obj)
            matrix = [obj.dotAnglesWaist obj.dotAnglesHip obj.dotAnglesKnee];
        end
        function matrix = to_vector_dot_end_positions(obj)
            matrix = [obj.dotEndPositions];
        end
        function matrix = to_vector_dot_end_positions_flatten(obj)
            matrix = [obj.dotEndPositions(1,:) obj.dotEndPositions(2,:) obj.dotEndPositions(3,:) obj.dotEndPositions(4,:)];
        end
        function matrix = to_vector_planned_end_positions_flatten(obj)
            matrix = [obj.plannedEndPositions(1,:) obj.plannedEndPositions(2,:) obj.plannedEndPositions(3,:) obj.plannedEndPositions(4,:)];
        end
        function matrix = to_vector_cond(obj)
            matrix = [obj.condA1Leg obj.condA2Leg obj.condA1Body obj.condA2Body];
        end
    end
end