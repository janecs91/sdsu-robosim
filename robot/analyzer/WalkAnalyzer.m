classdef WalkAnalyzer < CostAnalyzer
    properties
    end
    methods
        function obj = WalkAnalyzer()
            obj@CostAnalyzer();
        end
        %% Time/Power Cost Evaluation Functions
        function [timeCosts, jointDistances, endPosDistances, timeJoints, timeWheels] = analyze_time(obj, bot, terrain)
            %[jointsMatrix, indexAfterDistance] = obj.get_joints_matrix_after_distance(bot);
            jointsMatrix = bot.jointsMatrix;
            [timePerJoint, jointChanges] = obj.get_joints_time_costs(jointsMatrix, terrain);
            timeCosts = max(timePerJoint,[],2);
            timeWheels = [];
            timeJoints = timeCosts;
            totalTimeCost = sum(timeCosts);
            jointDistances = max(jointChanges, [], 2);
            totalJointChanges = sum(jointDistances);
            if obj.verbose && false
                disp("max joint changes")
                disp(totalJointChanges)
                disp("*** max time costs ***")
                disp("time costs per joint")
                disp(timePerJoint)
                disp("max time joints")
                disp(timeCosts)
            end
            %cumulativeTimeCosts = cumsum(maxTimeCosts);
            %distance = squeeze(bot.baseMatrix(end, 1) - bot.baseMatrix(indexAfterDistance, 1));
            %avgVelocity = distance/(max(totalTimeCost, 1));
            endPosDistances = obj.get_end_position_distances(bot.endPositionsMatrix);
            if obj.verbose
                disp("end position distances");
                disp(endPosDistances);
            end
        end
        function powerCost = analyze_power(obj, bot, terrain)
            % take cost after distance 500cm (if greater)
            %jointsMatrix = obj.get_joints_matrix_after_distance(bot);
            jointsMatrix = bot.jointsMatrix;
            powerCostPerJoint = obj.get_joints_power(jointsMatrix, terrain);
            powerCost = sum(powerCostPerJoint,2);
            totalPowerCost = sum(powerCost);
        end
        %% WIP
        function [strideLengths, avgStrideLength] = analyze_stride_length(obj, bot, terrain)
            endPosDistances = obj.get_end_position_distances(bot.endPositionsMatrix);
            stateHistory = bot.stateHistory;
            disp(size(stateHistory))
            for i=2:size(stateHistory)
                activeLeg = stateHistory{i}.activeLeg;
                inactiveLegs = [1 2 3 4];
                endPosDistances(inactiveLegs) = 0;
            end
            disp("endPosDistances")
            disp(endPosDistances)
        end
    end
end