classdef StepAdjusterHeight < StepAdjuster
    properties
        originalBaseZ = 24;
        enableIncreaseHeight = true;
        enableDecreaseToOriginal = false;
        verbose = true;
    end
    methods
        function obj = StepAdjusterHeight(safetyValue)
            obj@StepAdjuster(safetyValue);
        end
        function dz_b = adjust_base_z_vector_for_swing(obj, bot, state, terrain)
            %disp("BASE Z SWING====");
            maxBaseZ = bot.get_max_height_from_waist_to_foot(state);
            requiredZ = StepAdjusterHeight.get_z_difference_between_feet(bot, state, terrain);
            dz_b = 0;
            if obj.verbose
                fprintf("height adj-- max base z: %.2f , req Z: %.2f, orig height: %.2f\n", maxBaseZ, requiredZ, obj.originalBaseZ);
            end
            if obj.enableIncreaseHeight == true && maxBaseZ < requiredZ
                %disp("maxB < reqZ");
                %dz_b = requiredZ;
                dz_b = requiredZ - maxBaseZ;
            end
            if obj.enableDecreaseToOriginal == true && requiredZ < obj.originalBaseZ && maxBaseZ > obj.originalBaseZ
                dz_b = obj.originalBaseZ - maxBaseZ;
            end
            if obj.verbose
                fprintf("step adjuster height -> dz_b: %.2f\n",dz_b);
            end
            %dz_b = 0;
        end
    end
    methods(Static)
        function differenceInZ = get_z_difference_between_feet(bot, state, terrain)
            %foot1 = terrain.get_elevation(state.endPositions(1,1),state.endPositions(1,2));
            %foot2 = terrain.get_elevation(state.endPositions(2,1),state.endPositions(2,2));
            elevations = terrain.get_elevations(state.endPositions(:,1),state.endPositions(:,2));
            %disp("terr elevations")
            %disp(elevations)
            maxFootElevation = max(elevations);
            minFootElevation = min(elevations);
            differenceInZ = abs(maxFootElevation - minFootElevation);
            %disp("z diff foots")
            %disp(maxFootElevation);
            %disp(minFootElevation);
        end
        
    end
end