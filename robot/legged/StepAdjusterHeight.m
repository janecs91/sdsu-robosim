classdef StepAdjusterHeight < StepAdjuster
    properties
        originalBaseHeight = 30;
        enableIncreaseHeight = true;
        enableDecreaseToOriginal = true;
        verbose = false;
    end
    methods
        function obj = StepAdjusterHeight(safetyValue)
            obj@StepAdjuster(safetyValue);
        end
        function dz_b = adjust_base_z_vector_for_swing(obj, bot, state, terrain)
            %disp("BASE Z SWING====");
            elevations = terrain.get_elevations(state.endPositions(:,1),state.endPositions(:,2));
            fakeState = copy(state);
            fakeState.endPositions(:,3) = elevations;
            maxBaseZ = bot.get_max_height_from_waist_to_foot(fakeState);
            requiredZ = StepAdjusterHeight.get_z_difference_between_feet(bot, state, terrain);
            dz_b = 0;
            if obj.verbose
                fprintf("height adj-- max base z: %.2f , req Z: %.2f, orig height: %.2f\n", maxBaseZ, requiredZ, obj.originalBaseHeight);
            end
            if obj.enableIncreaseHeight == true && maxBaseZ < requiredZ
                %disp("maxB < reqZ");
                %dz_b = requiredZ;
                dz_b = requiredZ - maxBaseZ;
            end
            if obj.enableDecreaseToOriginal == true && requiredZ < obj.originalBaseHeight && maxBaseZ > obj.originalBaseHeight
                dz_b = obj.originalBaseHeight - maxBaseZ;
                if obj.verbose
                    fprintf("decrease height part 1 - dz_b: %d\n", dz_b)
                end

                averageElevation = bot.get_average_elevation(state, terrain);
                if state.basePosition(3)+dz_b < averageElevation+bot.minBottomZFromTerrain
                    dz_b = averageElevation+bot.minBottomZFromTerrain-state.basePosition(3);
                    if obj.verbose
                        fprintf("decrease height part 2 terrain check - dz_b: %d\n", dz_b)
                        fprintf("state base z: %d, average elevation: %d, min bottom z: %d\n", state.basePosition(3), averageElevation, bot.minBottomZFromTerrain);
                    end
                end
                
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