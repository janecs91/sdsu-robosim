%% Utility class
% general utility functions
classdef Utils
    methods(Static)
        %% sub_values_from_map: substitute constants or values from a map
        function input = sub_values_from_map(input, subMap)
            keySet = keys(subMap);
            if iscell(input)
                % preserves cell shape
                sizeCell = size(input);
                for i = 1:sizeCell
                    for k = 1:length(keySet)
                        key = keySet{k};
                        value = subMap(key);
                        input{i} = subs(input{i}, key, value);
                    end
                end
            else
                % subs() may flatten data if cell, but OK for input matrices and arrays
                for k = 1:length(keySet)
                    key = keySet{k};
                    value = subMap(key);
                    input = subs(input, key, value);
                end
            end
        end
    end
end