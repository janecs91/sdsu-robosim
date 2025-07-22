classdef WalkAnalyzer < CostAnalyzer
    properties
    end
    methods
        function obj = WalkAnalyzer()
            obj@CostAnalyzer();
        end
        %% Time/Power Cost Evaluation Functions
        function [totalTimeCost, totalJointChanges, timeCosts, velocity, distance] = analyze_time(obj, bot, terrain)
            [jointsMatrix, indexAfterDistance] = obj.get_joints_matrix_after_distance(bot);
            [timePerJoint, jointChanges] = obj.get_joints_time_costs(jointsMatrix, terrain);
            timeCosts = max(timePerJoint,[],2);
            totalTimeCost = sum(timeCosts);
            totalJointChanges = sum(max(jointChanges, [], 2));
            if obj.verbose
                disp("max joint changes")
                disp(totalJointChanges)
                disp("*** max time costs ***")
                disp("time costs per joint")
                disp(timePerJoint)
                disp("max time joints")
                disp(timeCosts)
            end
            %cumulativeTimeCosts = cumsum(maxTimeCosts);
            distance = squeeze(bot.baseMatrix(end, 1) - bot.baseMatrix(indexAfterDistance, 1));
            velocity = distance/(max(totalTimeCost, 1));
        end
        function maxPowerCost = analyze_power(obj, bot, terrain)
            % take cost after distance 500cm (if greater)
            jointsMatrix = obj.get_joints_matrix_after_distance(bot);
            maxPowerCost = obj.get_joints_power(jointsMatrix, terrain);
        end
    end
end