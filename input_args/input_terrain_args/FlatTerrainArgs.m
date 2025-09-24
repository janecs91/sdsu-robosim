classdef FlatTerrainArgs < TerrainArgsInterface
    methods
        function obj = FlatTerrainArgs()
            %% terrain settings
            obj.terrainName = 'flat';
            obj.genTerrainFromPath = true;
            % custom, ignore if gen from path
            %obj.width = pathArgs.pathEndX+100;
            %obj.height = pathArgs.pathEndY+200;
            obj.cellsize = 5;
            % sin
            obj.terrainAmplitude = 13;
            obj.terrainFrequency = 1;
            % random
            obj.randFilterSize = 10;
            obj.elevationChangeRange = 14;
            %terrainName = 'customTerrainName'; 
        end
    end
end