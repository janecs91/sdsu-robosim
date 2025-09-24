classdef TerrainArgsInterface
    properties(Constant)
        terrainOptions = {'flat', 'ramp', 'sin', 'random', 'leftright', 'halfsin',   'perlin'};
    end
    properties
        terrainName
        genTerrainFromPath
        width
        height
        cellsize
        terrainAmplitude
        terrainFrequency
        randFilterSize
        elevationChangeRange
    end
    methods
        function obj = TerrainArgsInterface()
            %% terrain settings
            % Abstract interface
        end
        function obj = setTerrainSize(obj, width, height)
            obj.width = width;
            obj.height = height;
        end

        function terrainArgs = getTerrainArgs(obj, pathName)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            terrainRequiredArgs = {obj.genTerrainFromPath, pathName, obj.terrainName};
            terrainExtraArgs = {obj.width, obj.height, obj.cellsize, obj.terrainAmplitude, obj.terrainFrequency, obj.randFilterSize, obj.elevationChangeRange};
            terrainArgs = [terrainRequiredArgs terrainExtraArgs];
        end
    end
end