classdef CostAnalyzer
    properties(Constant)
        % segment feature (for terrain roughness)
        segmentSize = 5;    % path length, in meters
        numPoints = 4;     % # points to sample
        % Joints: [waist hip knee steering]
        % Velocity Units: in radians/sec
        %maxJointVelocitiesPaperOld = [1.02 19.82 20.57];   % from paper (old version, did not count 2nd gears for hip, knee)
        %maxJointVelocitiesPaperWSteerOld = [1.02 19.82 20.57 20.57];  % from paper old vers, includes steering
        maxJointVelocitiesPaper = [1.02 0.96 1.];
        maxWheelVelocityPaper = 1.6;   % from paper
        %maxJointVelocitiesAdjusted = [1. 1. 1.];
        %maxWheelVelocityAdjusted = 26.50;
        %maxWheelVelocityAdjusted = 0.8; - was latest
        maxWheelVelocityAdjusted = 2;   % from rollbot
        % Power Units: in Watts
        maxJointPowersPaper = [13.17 70.76 25.49];      % from paper
        maxJointPowersPaperWSteer = [13.17 70.76 25.49 25.49];  % from paper, includes steering
        maxWheelPowerPaper = 25.49;     % from paper
        maxWheelPowerAdjusted = 36.18;

        verbose = false;
    end
    properties
        % joint/wheel properties can be adjusted here
        maxJointVelocities = CostAnalyzer.maxJointVelocitiesPaper;
        maxWheelVelocity = CostAnalyzer.maxWheelVelocityAdjusted;
        maxJointPowers = CostAnalyzer.maxJointPowersPaper;
        maxWheelPower = CostAnalyzer.maxWheelPowerAdjusted;
        % costs
        ignoreCostsUntilAfterDistance = -1;
    end
    methods
        function obj = CostAnalyzer()
        end
        %% Steady State Analysis
        function index = get_steady_state(obj, baseMatrix)
            index = 0;
        end
        function velocity = get_steady_state_velocity(obj, baseMatrix)
            velocity = 0;
            % time/distance
            % time
            % distance
        end
        %% Time Analysis
        function dotJointsMatrix = get_joints_changes(obj, jointsMatrix)
            dotJointsMatrix = jointsMatrix(2:end,:) - jointsMatrix(1:end-1,:);
            dotJointsMatrix = abs(dotJointsMatrix);
            %jointChanges = (1+sinkRate)*dotJointsMatrix;
        end
        function [timeCostsPerJoint, jointChanges] = get_joints_time_costs(obj, jointsMatrix, terrain)
            % time cost for joints
            sinkRate = terrain.sinkRate;
            jointChanges = obj.get_joints_changes(jointsMatrix);
            affectedJointChanges = (1+sinkRate)*jointChanges;
            timeCostsPerJoint = affectedJointChanges ./ repelem(obj.maxJointVelocities, 4);
            if obj.verbose && false
                disp("affected joint changes")
                disp(affectedJointChanges)
                disp("time costs per joint")
                disp(timeCostsPerJoint)
            end
        end
        function distances = get_end_position_distances(obj, endPositionsMatrix)
            dotEndPositionsMatrix = endPositionsMatrix(:,:,2:end) - endPositionsMatrix(:,:,1:end-1);
            distances = abs(sqrt(sum(dotEndPositionsMatrix.^2,2)));
            distances = squeeze(distances);
            if obj.verbose
                disp("wheels - dotEndPositionsMatrix")
                disp(dotEndPositionsMatrix)
                disp("wheels - distances")
                disp(distances)
            end
        end
        function [timeCostsPerWheel, distances] = get_wheels_time_costs(obj, endPositionsMatrix, terrain, wheelRadius)
            distances = obj.get_end_position_distances(endPositionsMatrix);
            
            % velocity based on traversal difficulty
            epsilon = 0.003;
            mu = 4;
            difficulty = obj.get_terrain_traversal_difficulty(terrain, endPositionsMatrix, epsilon);
            stateVelocity = ((1+epsilon)*wheelRadius*obj.maxWheelVelocity)./(((1+epsilon)*wheelRadius)+((mu-1).*difficulty));
            
            % get slip rates
            slipRates = zeros(4, size(distances,2));
            numLegs = size(endPositionsMatrix, 1);
            for leg=1:numLegs
                xPositions = endPositionsMatrix(leg,1,2:end);
                yPositions = endPositionsMatrix(leg,2,2:end);
                slipRates(leg,:) = terrain.get_slip_rates(xPositions, yPositions);
            end
            
            actualVelocity = ((1-slipRates).*(2*pi*wheelRadius)).*stateVelocity;
            timeCostsPerWheel = distances./actualVelocity;
            if obj.verbose && true
                disp("state velocity")
                disp(stateVelocity)
                disp("actual velocity")
                disp(actualVelocity)
                disp("time costs per wheel")
                disp(timeCostsPerWheel)
            end
        end
        %% Power Analysis
        function powerCost = get_joints_power(obj, jointsMatrix, terrain)
            timeCostsMatrix = obj.get_joints_time_costs(jointsMatrix, terrain);
            powerCost = timeCostsMatrix .* repelem(obj.maxJointPowers, 4);
            %totalPowerCost = sum(powerCost,'all');
        end
        function powerCost = get_wheels_power(obj, endPositionsMatrix, wheelRadius, terrain)
            timeCostsMatrix = obj.get_wheels_time_costs(endPositionsMatrix, terrain, wheelRadius);
            powerCost = timeCostsMatrix.*obj.maxWheelPower;
            %totalPowerCost = sum(powerCost,'all');
        end
        %% Terrain Traversal Difficulty
        function difficulty = get_terrain_traversal_difficulty(obj, terrain, endPositions, epsilon)
            % only for line
            %disp('traversal difficulty');
            pointDistance = obj.segmentSize/obj.numPoints;
            halfNumPoints = obj.numPoints/2;
            numLegs = size(endPositions, 1);
            numStates = size(endPositions, 3);
            differences = zeros(numLegs, numStates-1, obj.numPoints-1);
            slopes = zeros(numLegs, numStates-1);
            for i = 1:numStates-1 % states
                for leg = 1:numLegs % legs 1-4
                    x = endPositions(leg, 1, i+1);
                    y = endPositions(leg, 2, i+1);
                    xPoints = x-floor(halfNumPoints):pointDistance:x+ceil(halfNumPoints);
                    yPoints = y-floor(halfNumPoints):pointDistance:y+ceil(halfNumPoints);
                    % get relevant elevation points
                    elevationPoints = terrain.get_elevations(xPoints, yPoints);
                    % segment
                    differences(leg, i, :) = abs(elevationPoints(2:end) - elevationPoints(1:end-1));
                    slopes(leg, i) = ((elevationPoints(end)-elevationPoints(1))/obj.numPoints);
                end
            end
            % difficulty
            sum_of_all_differences = sum(differences, 3);
            difficulty = sum_of_all_differences + slopes*epsilon;
        end
        
        % used in cost analysis
        function [matrix, index] = get_joints_matrix_after_distance(obj, bot, distance)
            if nargin < 3
                distance = obj.ignoreCostsUntilAfterDistance;
            end
            matrix = bot.jointsMatrix;
            index = find(bot.baseMatrix(:, 1) > distance);
            if size(matrix, 1) > distance
                matrix = matrix(index, :);
            end
            index = index(1);
        end
        function [matrix, index] = get_end_positions_matrix_after_distance(obj, bot, distance)
            if nargin < 3
                distance = obj.ignoreCostsUntilAfterDistance;
            end
            matrix = bot.endPositionsMatrix;
            index = find(bot.baseMatrix(:, 1) > distance);
            if size(matrix, 1) > distance
                matrix = matrix(:, :, index);
            end
            index = index(1);
        end
    end
    methods(Static)
        
    end
end