classdef RollAnalyzer < CostAnalyzer
    methods
        function obj = RollAnalyzer()
            obj@CostAnalyzer();
        end
        
        %% Time/Power Cost Evaluation Functions
        function [timeCosts, jointDistances, wheelDistances, timeJoints, timeWheels] = analyze_time(obj, bot, terrain)
            %[jointsMatrix, indexAfterDistance] = obj.get_joints_matrix_after_distance(bot, obj.ignoreCostsUntilAfterDistance);
            %[endPositionsMatrix, indexAfterDistance] = obj.get_end_positions_matrix_after_distance(bot, obj.ignoreCostsUntilAfterDistance);

            jointsMatrix = bot.jointsMatrix;
            endPositionsMatrix = bot.endPositionsMatrix;
            
            [timePerJoint, jointChanges] = obj.get_joints_time_costs(jointsMatrix, terrain);
            [timePerWheel, distances] = obj.get_wheels_time_costs(endPositionsMatrix, terrain, bot.wheelRadius);
            timeWheels = max(timePerWheel.',[],2);
            timeJoints = max(timePerJoint,[],2);
            timeCosts = max(timeJoints, timeWheels);
            if obj.verbose && false
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
            jointDistances = max(jointChanges, [], 2);
            totalJointChanges = sum(jointDistances);
            wheelDistances = max(distances, [], 1);
            totalWheelDistance = sum(wheelDistances);
            avgVelocity = totalWheelDistance/(max(totalTimeCost, 1));
        end
        function powerCost = analyze_power(obj, bot, terrain)
            % extend dotJointsMatrix to include knee, ankle
            jointPowerCost = obj.get_joints_power(bot.jointsMatrix, terrain);
            wheelPowerCost = obj.get_wheels_power(bot.endPositionsMatrix, bot.wheelRadius, terrain);
            sumJointPowerCost = sum(jointPowerCost,2);
            sumWheelPowerCost = sum(wheelPowerCost,1)';
            %size(sumJointPowerCost)
            %size(sumWheelPowerCost)
            powerCost = sumJointPowerCost+sumWheelPowerCost;
            totalPowerCost = sum(powerCost);
            disp(powerCost)
            disp(totalPowerCost)
        end
    end
end