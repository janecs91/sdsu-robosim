classdef RollAnalyzer < CostAnalyzer
    methods
        function obj = RollAnalyzer()
            obj@CostAnalyzer();
        end
        
        %% Time/Power Cost Evaluation Functions
        function [totalTimeCost, totalJointChanges, timeCosts, velocity, wheelDistance] = analyze_time(obj, bot, terrain)
            [jointsMatrix, indexAfterDistance] = obj.get_joints_matrix_after_distance(bot, obj.ignoreCostsUntilAfterDistance);
            [endPositionsMatrix, indexAfterDistance] = obj.get_end_positions_matrix_after_distance(bot, obj.ignoreCostsUntilAfterDistance);
            
            [timePerJoint, jointChanges] = obj.get_joints_time_costs(jointsMatrix, terrain);
            [timePerWheel, distances] = obj.get_wheels_time_costs(endPositionsMatrix, terrain, bot.wheelRadius);
            timeWheels = max(timePerWheel.',[],2);
            timeJoints = max(timePerJoint,[],2);
            timeCosts = max(timeJoints, timeWheels);
            if obj.verbose
                disp("*** distances ***")
                disp(distances)
                disp("*** max time costs ***")
                disp("max time wheels")
                disp(timeWheels)
                disp("max time joints")
                disp(timeJoints)
                disp("max time overall")
                disp(timeCosts)
            end
            %cumulativeTimeCosts = cumsum(maxTimeCosts);
            totalTimeCost = sum(timeCosts);
            totalJointChanges = sum(max(jointChanges, [], 2));
            wheelDistance = sum(max(distances, [], 1));
            velocity = wheelDistance/(max(totalTimeCost, 1));
        end
        function maxPowerCost = analyze_power(obj, bot, terrain)
            % extend dotJointsMatrix to include knee, ankle
            maxPowerCost = obj.get_joints_power(bot.jointsMatrix, terrain) + ...
                obj.get_wheels_power(bot.endPositionsMatrix, bot.wheelRadius, terrain);
        end
    end
end